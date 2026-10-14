"""Compare current firmware float QPR with the supplied SLX double recurrence.

SLX XML numeric expressions are parsed with a restricted AST, never executed.
Both actual current-PR paths are exercised without output saturation. The main
path's outer-loop gains are set to zero in the host shim solely to inject a
known current error; the firmware source is compiled without modification.
"""
from pathlib import Path
import _ctypes
import argparse
import ast
import ctypes
import hashlib
import json
import math
import re
import struct
import subprocess
import tempfile
import xml.etree.ElementTree as ET
import zipfile

ROOT = Path(__file__).resolve().parents[2]
F32 = lambda v: struct.unpack('<f', struct.pack('<f', v))[0]
SHIM = r'''
#include "qpr_control.h"
static QPR_Control master, slave;
static float audit_bus;
int audit_init(float fs) {
    int ok=QPR_ControlInit(&master,fs) && QPR_ControlInit(&slave,fs);
    master.virtual_b=0; master.virtual_decay=0;
    audit_bus=48;
    return ok;
}
void audit_set_bus(float bus) { audit_bus=bus; }
void audit_step(float error,float *out) {
    float old_r=slave.resonant_r, old_q=slave.resonant_q;
    QPR_ControlStep(&master,0,0,0,-error,48);
    QPR_CurrentStep(&slave,error,0,0,audit_bus);
    out[0]=master.qpr_unlimited_v; out[1]=slave.qpr_unlimited_v;
    out[2]=(float)master.fault_flags; out[3]=(float)slave.fault_flags;
    out[4]=(float)master.qpr_saturation_count;
    out[5]=(float)slave.qpr_saturation_count;
    out[6]=(float)((error!=0) && (old_r==slave.resonant_r) && (old_q==slave.resonant_q));
    out[7]=error*(slave.bridge_reference_v-slave.modulation*audit_bus);
}
'''


def arithmetic(expression, env):
    node=ast.parse(expression.strip().replace('^','**'),mode='eval').body
    def evaluate(n):
        if isinstance(n, ast.Constant) and type(n.value) in (int,float): return float(n.value)
        if isinstance(n, ast.Name) and n.id in env: return env[n.id]
        if isinstance(n, ast.UnaryOp) and isinstance(n.op,(ast.UAdd,ast.USub)):
            return evaluate(n.operand)*(-1 if isinstance(n.op,ast.USub) else 1)
        if isinstance(n,ast.BinOp):
            a,b=evaluate(n.left),evaluate(n.right)
            if isinstance(n.op,ast.Add): return a+b
            if isinstance(n.op,ast.Sub): return a-b
            if isinstance(n.op,ast.Mult): return a*b
            if isinstance(n.op,ast.Div): return a/b
            if isinstance(n.op,ast.Pow): return a**b
        raise ValueError('Unsupported numeric expression: '+expression)
    return evaluate(node)


def read_model(path):
    with zipfile.ZipFile(path) as z:
        block=ET.fromstring(z.read('simulink/systems/system_241.xml')).find("./Block[@SID='4']")
        assert block.get('Name')=='Digital PR'
        values={p.get('Name'):arithmetic(p.findtext('Value'),{})
                for p in block.findall('./Mask/MaskParameter')}
        script=ET.fromstring(z.read('simulink/stateflow/chart_26.xml')).find(".//P[@Name='script']").text
        env=dict(values,pi=math.pi)
        for name in ['W0','t1','t2','t3','b0','b1','b2','a1','a2']:
            match=re.search(r'^\s*'+name+r'\s*=\s*([^;\r\n]+);',script,re.M)
            assert match, name
            env[name]=arithmetic(match.group(1),env)
        out=re.search(r'^\s*out_A\s*=\s*([^;\r\n]+);',script,re.M).group(1)
        assert re.sub(r'\s+','',out)=='b0*err_A+b1*err_A1+b2*err_A2-a1*out_A1-a2*out_A2'
        assert abs(env['b0'])<3000
        return dict(parameters=values, coefficients={k:env[k] for k in ['b0','b1','b2','a1','a2']},
                    model_sha256=hashlib.sha256(path.read_bytes()).hexdigest(),
                    chart_script_sha256=hashlib.sha256(script.encode()).hexdigest(),
                    chart='simulink/stateflow/chart_26.xml', mask='simulink/systems/system_241.xml SID4')


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--model',type=Path,default=ROOT.parent.parent/'MATLAB-study/BINLIANSinglePhase.slx')
    p.add_argument('--compiler',default='D:/tools/mingw64/bin/gcc.exe')
    p.add_argument('--output',type=Path,default=Path(__file__).with_name('current_pr_model_results.json'))
    args=p.parse_args()
    model=read_model(args.model)
    ts=model['parameters']['Ts']
    co=model['coefficients']
    result=dict(scope='Current QPR C compiled unchanged; supplied SLX XML double recurrence. Isolated PR numerical equivalence only; not a closed-loop plant or hardware test.',
                model=model,input_amplitude_a=0.1,simulation_duration_s=10,comparison_tail_s=1,
                source_sha256=hashlib.sha256((ROOT/'Core/Src/qpr_control.c').read_bytes()).hexdigest(),
                header_sha256=hashlib.sha256((ROOT/'Core/Inc/qpr_control.h').read_bytes()).hexdigest(),
                cases=[])
    with tempfile.TemporaryDirectory(prefix='current_pr_model_') as tmp:
        src,dll=Path(tmp)/'bridge.c',Path(tmp)/'pr.dll'
        src.write_text(SHIM,encoding='utf-8')
        subprocess.run([args.compiler,'-std=c99','-O2','-Wall','-Wextra','-Werror','-shared',
                        '-I'+str(ROOT/'Core/Inc'),str(src),str(ROOT/'Core/Src/qpr_control.c'),'-o',str(dll)],check=True)
        lib=ctypes.CDLL(str(dll))
        lib.audit_init.argtypes=[ctypes.c_float]
        lib.audit_init.restype=ctypes.c_int
        lib.audit_step.argtypes=[ctypes.c_float,ctypes.POINTER(ctypes.c_float)]
        lib.audit_set_bus.argtypes=[ctypes.c_float]
        output=(ctypes.c_float*8)()
        for hz,bus in [(49,48),(50,48),(51,48),(49,64),(50,64),(51,64)]:
            assert lib.audit_init(1.0/ts)==1
            lib.audit_set_bus(bus)
            e1=e2=y1=y2=0.0
            squared=[0.0,0.0]
            maxdiff=[0.0,0.0]
            maxoutput=[0.0,0.0]
            count=0
            unchanged_state_count=positive_roundtrip_count=0
            max_roundtrip=0.0
            for n in range(round(10/ts)):
                e=F32(0.1*math.sin(2*math.pi*hz*n*ts))
                expected=co['b0']*e+co['b1']*e1+co['b2']*e2-co['a1']*y1-co['a2']*y2
                e2,e1,y2,y1=e1,e,y1,expected
                lib.audit_step(e,output)
                assert output[2]==output[3]==output[4]==output[5]==0
                unchanged_state_count+=int(output[6])
                positive_roundtrip_count+=int(output[7]>0)
                max_roundtrip=max(max_roundtrip,abs(output[7]))
                for k in [0,1]:
                    maxoutput[k]=max(maxoutput[k],abs(output[k]))
                    if n>=round(9/ts):
                        delta=float(output[k])-expected
                        squared[k]+=delta*delta
                        maxdiff[k]=max(maxdiff[k],abs(delta))
                if n>=round(9/ts): count+=1
            errors=[math.sqrt(x/count) for x in squared]
            primary_passed=errors[0]<1e-5 and maxdiff[0]<3e-5
            slave_passed=errors[1]<1e-5 and maxdiff[1]<3e-5
            case=dict(input_frequency_hz=hz,slave_bus_v=bus,
                      main_current_pr_rms_difference_v=errors[0],main_current_pr_max_difference_v=maxdiff[0],
                      slave_current_step_rms_difference_v=errors[1],slave_current_step_max_difference_v=maxdiff[1],
                      main_peak_output_v=maxoutput[0],slave_peak_output_v=maxoutput[1],
                      faults=0,saturation_count=0,primary_check_passed=primary_passed,
                      slave_diagnostic_equivalence_passed=slave_passed,
                      slave_unchanged_resonant_state_steps=unchanged_state_count,
                      slave_positive_roundtrip_product_steps=positive_roundtrip_count,
                      slave_max_abs_roundtrip_product_a_v=max_roundtrip)
            result['cases'].append(case)
            print(f'{hz}Hz bus={bus}V: main RMS delta={errors[0]:.9g}V max={maxdiff[0]:.9g}V; '
                  f'QPR_CurrentStep RMS delta={errors[1]:.9g}V max={maxdiff[1]:.9g}V; '
                  f'unchanged_state={unchanged_state_count} positive_roundtrip={positive_roundtrip_count}')
        _ctypes.FreeLibrary(lib._handle)
        del lib
    result['primary_equivalence_checks_passed']=all(c['primary_check_passed'] for c in result['cases'])
    result['slave_diagnostic_equivalence_checks_passed']=all(c['slave_diagnostic_equivalence_passed'] for c in result['cases'])
    result['slave_diagnostic_note']='No correction/modulation saturation occurred. At 48 V, division/multiplication roundoff creates a positive antiwindup product and freezes resonant state. At power-of-two 64 V the round trip is exact and the diagnostic matches the recurrence. Slave is disabled in the reported primary no-load condition.'
    args.output.write_text(json.dumps(result,ensure_ascii=False,indent=2)+'\n',encoding='utf-8')
    print(('PASS' if result['primary_equivalence_checks_passed'] else 'FAIL')+' supplied SLX recurrence vs main current PR')
    print(('PASS' if result['slave_diagnostic_equivalence_checks_passed'] else 'FAIL')+' separate slave diagnostic (not the primary no-load root cause)')
    print(args.output)


if __name__=='__main__': main()
