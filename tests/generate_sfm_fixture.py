#!/usr/bin/env python3
"""Create synthetic 3D multi-plane monocular benchmark videos.

No user footage, external downloads, or hidden reference datasets.
The camera moves in front of independently textured planes at different depths.
This is a functional integration test, NOT evidence of real-scene SLAM accuracy.
"""
import argparse
import subprocess
from pathlib import Path

import cv2
import numpy as np

def plane_texture(seed, w=1000, h=800):
    rng=np.random.default_rng(seed)
    im=np.full((h,w,3),210,dtype=np.uint8)
    im=np.clip(im.astype(np.int16)+rng.normal(0,18,(h,w,1)),0,255).astype(np.uint8)
    for _ in range(450):
        x,y=int(rng.integers(12,w-12)),int(rng.integers(12,h-12))
        color=tuple(int(t) for t in rng.integers(25,235,3))
        shape=int(rng.integers(0,3))
        if shape==0:cv2.circle(im,(x,y),int(rng.integers(2,9)),color,-1)
        elif shape==1:cv2.rectangle(im,(x,y),(x+int(rng.integers(2,25)),y+int(rng.integers(2,20))),color,2)
        else:cv2.line(im,(x,y),(x+int(rng.integers(-15,16)),y+int(rng.integers(-15,16))),color,2)
    cv2.putText(im,'PAI-VIN TEST '+str(seed),(30,h//2),cv2.FONT_HERSHEY_SIMPLEX,2,(30,30,30),4)
    return im

def draw(camera_x,camera_y,yaw,planes,k):
    height,width=480,640
    canvas=np.full((height,width,3),235,dtype=np.uint8)
    xx,yy=np.meshgrid(np.arange(width),np.arange(height))
    canvas[:,:,0]=np.clip(230+(yy-240)*0.025,0,255).astype('uint8')
    zbuf=np.full((height,width),np.inf)
    center=np.array([camera_x,camera_y,0.])
    c,s=np.cos(yaw),np.sin(yaw)
    R=np.array([[c,0,s],[0,1,0],[-s,0,c]],dtype=float)
    for tex,corners,depth in sorted(planes,key=lambda v:v[2],reverse=True):
        source=np.float32([[0,0],[tex.shape[1]-1,0],[tex.shape[1]-1,tex.shape[0]-1],[0,tex.shape[0]-1]])
        uv=[]
        for xyz in corners:
            p=R.T @ (xyz-center)
            uv.append([(k*p[0]/p[2])+width/2,(k*p[1]/p[2])+height/2])
        H=cv2.getPerspectiveTransform(source,np.float32(uv))
        warped=cv2.warpPerspective(tex,H,(width,height))
        mask=cv2.warpPerspective(np.full(tex.shape[:2],255,dtype='uint8'),H,(width,height))
        valid=(mask>0)&(depth<zbuf)
        canvas[valid]=warped[valid]
        zbuf[valid]=depth
    return canvas

def main():
    ap=argparse.ArgumentParser()
    ap.add_argument('--out',type=Path,required=True)
    ap.add_argument('--frames',type=int,default=36)
    args=ap.parse_args()
    out=args.out.resolve();out.mkdir(parents=True,exist_ok=True)
    planes=[]
    for i,(x,y,z,spanx,spany) in enumerate([
            (0.,0.,8.,10.,5.5),(-2.5,-0.3,5.5,1.7,2.0),
            (2.6,0.2,6.1,1.4,1.9),(0.3,1.5,4.6,1.3,0.9),
            (-0.2,-1.5,5.1,1.3,0.8)]):
        corners=np.array([[x-spanx/2,y-spany/2,z],[x+spanx/2,y-spany/2,z],
                          [x+spanx/2,y+spany/2,z],[x-spanx/2,y+spany/2,z]],dtype=float)
        planes.append((plane_texture(i+101),corners,z))
    for role,count,offset in [('mapping',args.frames,0),('localization',max(12,args.frames//3),0.08)]:
        folder=out/role;folder.mkdir(exist_ok=True)
        for i in range(count):
            u=i/max(1,count-1)
            # Both passes image the same static 3D scene from overlapping camera positions.
            x=-0.75+1.5*u+offset
            y=0.09*np.sin(u*5.0)+offset*0.2
            yaw=0.045*np.sin(u*6.0)+offset*0.1
            img=draw(x,y,yaw,planes,700.)
            cv2.imwrite(str(folder/f'{i:06d}.jpg'),img,[cv2.IMWRITE_JPEG_QUALITY,93])
        video=out/(role+'.mp4')
        subprocess.run(['ffmpeg','-hide_banner','-loglevel','error','-y',
            '-framerate','5','-i',str(folder/'%06d.jpg'),'-c:v','libx264',
            '-pix_fmt','yuv420p',str(video)],check=True)
        assert video.is_file() and video.stat().st_size>5000,video
    print('Generated synthetic test videos:',out/'mapping.mp4',out/'localization.mp4')

if __name__=='__main__':main()
