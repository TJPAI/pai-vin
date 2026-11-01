#!/usr/bin/env python3
"""PAI-VIN reproducible video preflight and frame export (no SLAM inference).
Requires installed ffmpeg and ffprobe; Python standard library only.
Usage: python3 tools/prepare_slam_dataset.py --mapping map.mp4 --localization loc.mp4 --out data/session01
"""
import argparse
import json
import math
import shutil
import subprocess
import sys
from pathlib import Path
from datetime import datetime, timezone

def cmd(args):
    p = subprocess.run(args, stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True)
    if p.returncode:
        raise RuntimeError('Command failed: ' + ' '.join(map(str, args)) + '\n' + p.stderr[-2500:])
    return p.stdout

def probe(video):
    result = json.loads(cmd(['ffprobe', '-v', 'error', '-show_streams', '-show_format', '-of', 'json', str(video)]))
    tracks = [s for s in result.get('streams', []) if s.get('codec_type') == 'video']
    if not tracks:
        raise ValueError('No video stream in ' + str(video))
    t = tracks[0]
    fps_raw = t.get('avg_frame_rate') or t.get('r_frame_rate') or '0/1'
    parts = fps_raw.split('/')
    fps = (float(parts[0]) / float(parts[1])) if len(parts) == 2 and float(parts[1]) else 0
    return dict(codec=t.get('codec_name'),width=t.get('width'),height=t.get('height'),
        fps=round(fps,3),duration_s=round(float(result.get('format',{}).get('duration',0)),3),
        video_bytes=video.stat().st_size,rotation_degrees=t.get('tags',{}).get('rotate'),
        pixel_format=t.get('pix_fmt'))

def extract(video, folder, fps, max_frames):
    folder.mkdir(parents=True,exist_ok=True)
    vf=f'fps={fps},scale=960:-2:flags=lanczos'
    cmd(['ffmpeg','-hide_banner','-loglevel','error','-y','-i',str(video),
         '-map','0:v:0','-vf',vf,'-frames:v',str(max_frames),
         '-q:v','3','-start_number','0',str(folder/'%06d.jpg')])
    names=sorted(folder.glob('*.jpg'))
    if not names: raise RuntimeError('No frames exported: '+str(video))
    (folder.parent/'timestamps.txt').write_text(''.join(f'{i/fps:.6f} {p.name}\n' for i,p in enumerate(names)),encoding='utf-8')
    return len(names)

def main():
    ap=argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--mapping',required=True,type=Path)
    ap.add_argument('--localization',required=True,type=Path)
    ap.add_argument('--out',required=True,type=Path)
    ap.add_argument('--fps',type=float,default=10)
    ap.add_argument('--max-frames',type=int,default=2500)
    args=ap.parse_args()
    for exe in ('ffmpeg','ffprobe'):
        if not shutil.which(exe): sys.exit(exe+' is required; install ffmpeg first')
    if not (1 <= args.fps <= 30) or args.max_frames<1:sys.exit('Invalid FPS or frame count')
    out=args.out.resolve();out.mkdir(parents=True,exist_ok=True)
    report={'schema':'pai-vin-slam-preflight-v1','generatedAt':datetime.now(timezone.utc).isoformat(),
      'target':'monocular image sequences for offline visual SLAM comparison',
      'status':'PREPROCESSING_ONLY_NOT_SLAM',
      'scale':'Unresolved. A4 board must be visually detected and jointly registered to SLAM camera poses.',
      'camera_intrinsics':'UNKNOWN: calibrate intrinsic matrix and lens distortion before metric claims',
      'datasets':{}}
    for role,video in (('mapping',args.mapping),('localization',args.localization)):
        video=video.resolve()
        if not video.is_file():sys.exit('Video missing: '+str(video))
        stat=probe(video)
        warnings=[]
        if stat['duration_s']<10:warnings.append('Too short for robust mapping/localization')
        if stat['fps']<15:warnings.append('Low capture FPS; motion blur and tracking gaps possible')
        if stat['width']<640:warnings.append('Low resolution')
        n=extract(video,out/role/'images',args.fps,args.max_frames)
        report['datasets'][role]={'source':str(video),'probe':stat,'export_fps':args.fps,
          'exported_frames':n,'frame_directory':str(out/role/'images'),
          'timestamps_file':str(out/role/'timestamps.txt'),'warnings':warnings}
    (out/'preflight.json').write_text(json.dumps(report,ensure_ascii=False,indent=2)+'\n',encoding='utf-8')
    print(json.dumps(report,ensure_ascii=False,indent=2))
    print('\nIMPORTANT: No SLAM map, 3D point cloud, trajectory, or localization accuracy has been computed.')

if __name__=='__main__':
    main()
