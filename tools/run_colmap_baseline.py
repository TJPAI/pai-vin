#!/usr/bin/env python3
"""Two-pass COLMAP monocular SfM baseline; requires external COLMAP binary.
Raw videos stay local. Mapping is frozen before localization registration.
This is offline SfM, not real-time mobile SLAM or metric localization.
"""
import argparse
import json
import shutil
import subprocess
import sys
from pathlib import Path

def execute(args, log):
    line=' '.join(map(str,args))
    print('RUN:',line,flush=True)
    with log.open('a',encoding='utf-8') as f:
        f.write('\n$ '+line+'\n')
        result=subprocess.run([str(a) for a in args],stdout=f,stderr=subprocess.STDOUT)
    if result.returncode:
        raise RuntimeError('Command failed with code '+str(result.returncode)+': '+line+'\nSee '+str(log))

def extract(video,output,fps,log):
    output.mkdir(parents=True,exist_ok=True)
    execute(['ffmpeg','-hide_banner','-y','-i',video,'-vf',f'fps={fps}',
             '-q:v','3','-start_number','0',output/'%06d.jpg'],log)
    names=sorted(output.glob('*.jpg'))
    if len(names)<10:raise RuntimeError('Too few frames extracted: '+str(output))
    return names

def run():
    ap=argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--mapping',type=Path,required=True)
    ap.add_argument('--localization',type=Path,required=True)
    ap.add_argument('--out',type=Path,required=True)
    ap.add_argument('--fps',type=float,default=5)
    ap.add_argument('--gpu',action='store_true',help='Use COLMAP SIFT GPU if available')
    ap.add_argument('--camera-model',default='SIMPLE_RADIAL')
    args=ap.parse_args()
    if args.fps<1 or args.fps>15:ap.error('--fps should be between 1 and 15')
    for tool in ['ffmpeg','colmap']:
        if not shutil.which(tool):sys.exit(tool+' executable missing. Install it on a workstation.')
    out=args.out.resolve()
    if out.exists() and any(out.iterdir()):sys.exit('Output directory is not empty; choose a new --out to avoid mixing experiments')
    out.mkdir(parents=True,exist_ok=True)
    log=out/'run.log'
    image_root=out/'images'
    mapping=extract(args.mapping.resolve(),image_root/'mapping',args.fps,log)
    query=extract(args.localization.resolve(),image_root/'localization',args.fps,log)
    db=out/'database.db';sparse=out/'sparse';sparse.mkdir()
    camera=['--ImageReader.camera_model',args.camera_model,
            '--ImageReader.single_camera','1']
    gpu='1' if args.gpu else '0'
    mapping_list=out/'mapping_images.txt'
    query_list=out/'localization_images.txt'
    mapping_list.write_text(''.join('mapping/'+p.name+'\n' for p in mapping),encoding='utf-8')
    query_list.write_text(''.join('localization/'+p.name+'\n' for p in query),encoding='utf-8')
    execute(['colmap','feature_extractor','--database_path',db,'--image_path',image_root,
             '--image_list_path',mapping_list,'--SiftExtraction.use_gpu',gpu,*camera],log)
    execute(['colmap','sequential_matcher','--database_path',db,'--SiftMatching.use_gpu',gpu],log)
    execute(['colmap','mapper','--database_path',db,'--image_path',image_root,'--output_path',sparse],log)
    recon=sorted([p for p in sparse.iterdir() if p.is_dir() and (p/'images.bin').exists()],
                 key=lambda p:int(p.name) if p.name.isdigit() else 999)
    if not recon:
        raise RuntimeError('COLMAP produced no sparse model. Inspect '+str(log))
    base=recon[0]
    (out/'mapping_model.txt').mkdir()\n    execute(['colmap','model_converter','--input_path',base,'--output_path',out/'mapping_cloud.ply',
             '--output_type','PLY'],log)
    execute(['colmap','model_converter','--input_path',base,'--output_path',out/'mapping_model.txt',
             '--output_type','TXT'],log)
    execute(['colmap','feature_extractor','--database_path',db,'--image_path',image_root,
             '--image_list_path',query_list,'--SiftExtraction.use_gpu',gpu,*camera],log)
    # All-pairs matching on this small benchmark (avoid mixing query frames into mapper).
    execute(['colmap','exhaustive_matcher','--database_path',db,'--SiftMatching.use_gpu',gpu],log)
    registered=out/'registered';registered.mkdir()
    execute(['colmap','image_registrator','--database_path',db,'--input_path',base,
             '--output_path',registered],log)
    (out/'registered_model.txt').mkdir()\n    execute(['colmap','model_converter','--input_path',registered,'--output_path',out/'registered_model.txt',
             '--output_type','TXT'],log)
    def image_count(path):
        p=path/'images.txt'
        if not p.exists():return None
        lines=[l for l in p.read_text().splitlines() if l and not l.startswith('#')]
        return len(lines)//2
    def registered_count(path,prefix):
        p=path/'images.txt'
        if not p.exists():return None
        lines=[l.strip().split() for l in p.read_text().splitlines() if l and not l.startswith('#')]
        return sum(len(line)>=10 and line[9].startswith(prefix) for line in lines[::2])
    n=registered_count(out/'registered_model.txt','localization/')
    report={'status':'offline_sfm_with_query_registration_attempted',
       'mapping_frames':len(mapping),'query_frames':len(query),
       'mapping_registered_frames':image_count(out/'mapping_model.txt'),
       'query_registered_frames':n,
       'query_registration_rate':None if n is None else round(n/len(query),4),
       'metric_scale_known':False,'p95_error_m':None,
       'notes':['Uncalibrated monocular intrinsics; arbitrary global scale',
         'COLMAP image_registrator registers query views against mapping model; it is not online SLAM.',
         'A4 board geometry not solved or registered; measured 0.5m error cannot be claimed.']}
    (out/'report.json').write_text(json.dumps(report,indent=2)+'\n',encoding='utf-8')
    print(json.dumps(report,indent=2))
if __name__=='__main__':run()
