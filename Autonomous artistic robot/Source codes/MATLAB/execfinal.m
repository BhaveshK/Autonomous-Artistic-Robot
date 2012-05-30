function [ ] = execfinal(img)
%  @author Group 6: Vineet K
%  
%  AVR Studio Version 4.17, Build 666
% 
%  
%  The aim of the project is to make the Spark V traverse along arbitrary
%  Shapes. This file is execfinal.m
%  It takes a image img as input and generates the required commandsfor the 
%  project
%   
% 
% *********************************************************************************/
% 
% /********************************************************************************
% 
%    Copyright (c) 2012, ERTS Lab IIT Bombay erts@cse.iitb.ac.in               -*- c -*-
%    All rights reserved.
% 
%    Redistribution and use in source and binary forms, with or without
%    modification, are permitted provided that the following conditions are met:
% 
%    * Redistributions of source code must retain the above copyright
%      notice, this list of conditions and the following disclaimer.
% 
%    * Redistributions in binary form must reproduce the above copyright
%      notice, this list of conditions and the following disclaimer in
%      the documentation and/or other materials provided with the
%      distribution.
% 
%    * Neither the name of the copyright holders nor the names of
%      contributors may be used to endorse or promote products derived
%      from this software without specific prior written permission.
% 
%    * Source code can be used for academic purpose. 
% 	 For commercial use permission form the author needs to be taken.
% 
%   THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
%   AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
%   IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
%   ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
%   LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
%   CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
%   SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
%   INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
%   CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
%   ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
%   POSSIBILITY OF SUCH DAMAGE. 
% 
%   Software released under Creative Commence cc by-nc-sa licence.
%   For legal information refer to: 
%   http://creativecommons.org/licenses/by-nc-sa/3.0/legalcode
% NTITLED4 Summary of this function goes here
%   Detailed explanation goes here


%img=imread('Final.bmp');
im=img;
% Red channel only
im_red=im(:,:,1);
im_gray=rgb2gray(im);
im_diffr=imsubtract(im_red,im_gray);
for m = 1:size(im_diffr,1)
      for n = 1:size(im_diffr,2)
          if im_diffr(m,n)>1
              im_diffr(m,n) = 255;
          end
      end
end
im_r = im2bw(im_diffr);
%im_r = bwmorph(im_diffr,'remove');

% green channel only
im_green=im(:,:,2);
im_gray=rgb2gray(im);
im_diffg=imsubtract(im_green,im_gray);
for m = 1:size(im_diffg,1)
      for n = 1:size(im_diffg,2)
          if im_diffg(m,n)>1
              im_diffg(m,n) = 255;
          end
      end
end
im_g = im2bw(im_diffg);
%im_g = bwmorph(im_diffg,'remove');

% blue channel only
im_blue=im(:,:,3);
im_gray=rgb2gray(im);
im_diffb=imsubtract(im_blue,im_gray);
for m = 1:size(im_diffb,1)
      for n = 1:size(im_diffb,2)
          if im_diffb(m,n)>1
              im_diffb(m,n) = 255;
          end
      end
end
im_b = im2bw(im_diffb);
%im_b = bwmorph(im_diffb,'remove');
figure,imshow(im);
%%
%%
%motion control red
I=im_r;
BW=im_r;
[B,L,N] = bwboundaries(BW);
%C=corner(BW,'QualityLevel',0.3);
figure; imshow(BW); hold on;
for k=1:length(B),
    boundary = B{k};
     plot(boundary(:,2),...
       boundary(:,1),'b','LineWidth',2);
end
hold on
%% Constants
Window = 3;
Sigma = 2;
K = 0.05;
nCorners = 100;

%% Derivative masks
dx = [-1 0 1; -1 0 1; -1 0 1];
dy = dx';   %SO code color fix '

%% Find the image gradient
Mask = im2bw(I);
Ix = conv2(double(Mask),dx,'same');   
Iy = conv2(double(Mask),dy,'same');

%% Use a gaussian windowing function and compute the rest
Gaussian = fspecial('gaussian',Window,Sigma);
Ix2 = conv2(Ix.^2,  Gaussian, 'same');  
Iy2 = conv2(Iy.^2,  Gaussian, 'same');
Ixy = conv2(Ix.*Iy, Gaussian, 'same');    

%% Find the corners
CornerStrength = (Ix2.*Iy2 - Ixy.^2) - K*(Ix2 + Iy2).^2;
[val ind] = sort(CornerStrength(:),'descend');    
[Ci Cj] = ind2sub(size(CornerStrength),ind(1:nCorners));
x=size(boundary);
for i=1:nCorners
    for j=1:x(1,1)
        if((Ci(i,1)==boundary(j,1))&&((Cj(i,1)==boundary(j,2))))
            break;
        end 
        if(j==x(1,1))
            Ci(i,1)=0;
            Cj(i,1)=0;
        end        
    end    
end    
%%
c=zeros(nCorners,2);
c(1,1)=Cj(1,1);
c(1,2)=Ci(1,1);
err=0;
for i=2:nCorners
    for j=1:nCorners 
        temp1=abs(c(j,1)-Cj(i,1));
        temp2=abs(c(j,2)-Ci(i,1));
        if(temp1<10&&temp2<10)
            err=1;
        end    
    end
    if(err==0)
        c(i,1)=Cj(i,1);
        c(i,2)=Ci(i,1);
    end 
    err=0;
end
plot(c(:,1), c(:,2), 'r*');

%%
g=zeros(nCorners,2);
s=0;
for k=1:x(1,1)
    for l=1:nCorners
        if((boundary(k,1)==c(l,2))&&(boundary(k,2)==c(l,1)))
            s=s+1;
            g(s,2)=boundary(k,1);
            g(s,1)=boundary(k,2);
            break;
        end    
    end
end   
for i=2:nCorners
    if(g(i,1)==g(i-1,1)&&g(i,2)==g(i-1,2))
        g(i,1)=0;
        g(i,2)=0;
    end    
end  
%%
j=2;
a=zeros(nCorners,1);
for i=1:nCorners
    if(g(i,1)==0&&g(i,2)==0)
        continue;
    end 
    a(j,1)=g(i,2);
    j=j+1;
end 
j=2;
b=zeros(nCorners,1);
for i=1:nCorners
    if(g(i,1)==0&&g(i,2)==0)
        continue;
    end
    b(j,1)=g(i,1);
    j=j+1;
end 
len=1;
for i=2:nCorners
    if(a(i,1)==0)
        break;
    end
    len=len+1;
end 
%%
s=serial('COM4');
fopen(s);
PI=3.14159265;
maxx=a(2);minx=0;maxy=b(2);miny=0;
   if(a(len-1)==a(len-2)&&b(len-1)==b(len-2))
        len=len-1;
   end     
   if((a(len)~=a(2))&&b(len)~=b(2))
   len=len+1 ;   
   a(len)=a(2);
   b(len)=b(2);
   end
    dx=maxx-minx;
    dy=maxy-miny;
    y=power(dy,2);
    x=power(dx,2);
	sum=x+y;
	dist=power(sum,0.5);
    tanangle=dy/dx;
    angle=atan(tanangle)*180/PI;
    
    angle=angle/10;
    angle=round(angle);
    angle=angle*10;
    ang=int2str(angle);
    ang=char(ang);
    la=length(ang);
    k=la-1;   
    fprintf(s,'r');
    pause(2); 
    fprintf(s,'s');
    pause(1);
    fprintf(s,'h');
    pause(1);
    fprintf(s,'1');
    pause(5);
    while(k>0)
        if(k==la-1)
            data=ang(k);
            fprintf(s,'r');
            pause(2); 
            fprintf(s,'a');
            pause(1);
            fprintf(s,'t');
            pause(1);
            fprintf(s,data);
            pause(5);
        end
        
        if(k==la-2)
            data=ang(k);
            fprintf(s,'r');
            pause(2); 
            fprintf(s,'a');
            pause(1);
            fprintf(s,'h');
            pause(1);
            fprintf(s,data);
            pause(5);  
        end 
        k=k-1;
    end    
    fprintf('left = %f',angle);
    fprintf('\n');
    fprintf(s,'r');
    pause(2); 
    fprintf(s,'w');
    pause(1);
    fprintf(s,'t');
    pause(1);
    fprintf(s,'4');
    pause(5);
    
    dist=dist/10;
    dist=round(dist);
    dist=dist*10;
    dis=int2str(dist);
    dis=char(dis);
    ld=length(dis);
    k=ld-1;
    while(k>0)
        if(k==ld-1)
            data=dis(k);
            fprintf(s,'r');
            pause(2); 
            fprintf(s,'w');
            pause(1);
            fprintf(s,'t');
            pause(1);
            fprintf(s,data);
            pause(5);
        end
        
        if(k==ld-2)
            data=dis(k);
            fprintf(s,'r');
            pause(2); 
            fprintf(s,'w');
            pause(1);
            fprintf(s,'h');
            pause(1);
            fprintf(s,data);
            pause(5);  
        end 
        k=k-1;
    end        
    fprintf('fwd = %f',round(dist)); 
    fprintf('\n');
    
    for i=2:len-1   
        minx=a(i);
        miny=b(i);
        maxx=a(i+1);
        maxy=b(i+1);
        dy=maxy-miny;
        dx=maxx-minx;
        l=distance(a(i),b(i),a(i-1),b(i-1));
        m=distance(a(i+1),b(i+1),a(i),b(i));
        n=distance(a(i+1),b(i+1),a(i-1),b(i-1));
        acosterm=(m*m+l*l-n*n)/(2*m*l);
        tempangle=acos(acosterm);
        tempangle=tempangle*180/PI;
        angle=180-tempangle;
        y=power(dy,2);
	    x=power(dx,2);
	    sum=x+y;
	    dist=power(sum,0.5);

        x1=a(i-1);y1=b(i-1);
        x2=a(i);y2=b(i);
        x3=a(i+1);y3=b(i+1);
	    dir=(x2-x1)*(y3-y1)-(x3-x1)*(y2-y1);
        
        angle=angle/10;
        angle=round(angle);
        angle=angle*10;
	    ang=int2str(angle);
        ang=char(ang);
        la=length(ang);
        k=la-1;
         fprintf(s,'r');
         pause(2); 
         fprintf(s,'s');
         pause(1);
         fprintf(s,'h');
         pause(1);
         fprintf(s,'1');
         pause(5);
        if(dir>0)
        while(k>0)
           if(k==la-1)
             data=ang(k);
             fprintf(s,'r');
             pause(2); 
             fprintf(s,'a');
             pause(1);
             fprintf(s,'t');
             pause(1);
             fprintf(s,data);
             pause(5);
           end
        
           if(k==la-2)
             data=ang(k);
             fprintf(s,'r');
             pause(2); 
             fprintf(s,'a');
             pause(1);
             fprintf(s,'h');
             pause(1);
             fprintf(s,data);
             pause(5);  
           end 
           k=k-1;
        end               
		   fprintf('left = %f',angle);
           fprintf('\n');
        end
        
        if(dir<0)
        while(k>0)
           if(k==la-1)
             data=ang(k);
             fprintf(s,'r');
             pause(2); 
             fprintf(s,'d');
             pause(1);
             fprintf(s,'t');
             pause(1);
             fprintf(s,data);
             pause(5);
           end
        
           if(k==la-2)
             data=ang(k); 
             fprintf(s,'r');
             pause(2); 
             fprintf(s,'d');
             pause(1);
             fprintf(s,'h');
             pause(1);
             fprintf(s,data);
             pause(5);  
           end 
           k=k-1;
        end              
		  fprintf('right = %f',angle);
          fprintf('\n');
        end
        fprintf(s,'r');
        pause(2); 
        fprintf(s,'w');
        pause(1);
        fprintf(s,'t');
        pause(1);
        fprintf(s,'4');
        pause(5);
        
        dist=dist/10;
        dist=round(dist);
        dist=dist*10;
	    dis=int2str(dist);
        dis=char(dis);
        ld=length(dis);
        k=ld-1;
        fprintf(s,'[');
        pause(2); 
        while(k>0)
           if(k==ld-1)
             data=dis(k); 
             fprintf(s,'r');
             pause(2); 
             fprintf(s,'w');
             pause(1);
             fprintf(s,'t');
             pause(1);
             fprintf(s,data);
             pause(5);
           end
        
           if(k==ld-2)
             data=dis(k); 
             fprintf(s,'r');
             pause(2); 
             fprintf(s,'w');
             pause(1);
             fprintf(s,'h');
             pause(1);
             fprintf(s,data);
             pause(5);  
           end 
           k=k-1;
        end          
        fprintf('fwd = %f',dist);fprintf('\n'); 
         fprintf(s,']');
        pause(2); 
    end 
fclose(s);
delete(s);
%%
%%
%motion control green

I=im_g;
BW=im_g;
[B,L,N] = bwboundaries(BW);
figure; imshow(BW); hold on;
for k=1:length(B),
    boundary = B{k};
     plot(boundary(:,2),...
       boundary(:,1),'b','LineWidth',2);
end
hold on
%% Constants
Window = 3;
Sigma = 2;
K = 0.05;
nCorners = 100;

%% Derivative masks
dx = [-1 0 1; -1 0 1; -1 0 1];
dy = dx';   %SO code color fix '

%% Find the image gradient
Mask = im2bw(I);
Ix = conv2(double(Mask),dx,'same');   
Iy = conv2(double(Mask),dy,'same');

%% Use a gaussian windowing function and compute the rest
Gaussian = fspecial('gaussian',Window,Sigma);
Ix2 = conv2(Ix.^2,  Gaussian, 'same');  
Iy2 = conv2(Iy.^2,  Gaussian, 'same');
Ixy = conv2(Ix.*Iy, Gaussian, 'same');    

%% Find the corners
CornerStrength = (Ix2.*Iy2 - Ixy.^2) - K*(Ix2 + Iy2).^2;
[val ind] = sort(CornerStrength(:),'descend');    
[Ci Cj] = ind2sub(size(CornerStrength),ind(1:nCorners));
x=size(boundary);
for i=1:nCorners
    for j=1:x(1,1)
        if((Ci(i,1)==boundary(j,1))&&((Cj(i,1)==boundary(j,2))))
            break;
        end 
        if(j==x(1,1))
            Ci(i,1)=0;
            Cj(i,1)=0;
        end        
    end    
end    
%%
c=zeros(nCorners,2);
c(1,1)=Cj(1,1);
c(1,2)=Ci(1,1);
err=0;
for i=2:nCorners
    for j=1:nCorners 
        temp1=abs(c(j,1)-Cj(i,1));
        temp2=abs(c(j,2)-Ci(i,1));
        if(temp1<10&&temp2<10)
            err=1;
        end    
    end
    if(err==0)
        c(i,1)=Cj(i,1);
        c(i,2)=Ci(i,1);
    end 
    err=0;
end
plot(c(:,1), c(:,2), 'r*');

%%
g=zeros(nCorners,2);
s=0;
for k=1:x(1,1)
    for l=1:nCorners
        if((boundary(k,1)==c(l,2))&&(boundary(k,2)==c(l,1)))
            s=s+1;
            g(s,2)=boundary(k,1);
            g(s,1)=boundary(k,2);
            break;
        end    
    end
end   
for i=2:nCorners
    if(g(i,1)==g(i-1,1)&&g(i,2)==g(i-1,2))
        g(i,1)=0;
        g(i,2)=0;
    end    
end  
%%
j=2;
a=zeros(nCorners,1);
for i=1:nCorners
    if(g(i,1)==0&&g(i,2)==0)
        continue;
    end 
    a(j,1)=g(i,2);
    j=j+1;
end 
j=2;
b=zeros(nCorners,1);
for i=1:nCorners
    if(g(i,1)==0&&g(i,2)==0)
        continue;
    end 
    b(j,1)=g(i,1);
    j=j+1;
end 
len=1;
for i=2:nCorners
    if(a(i,1)==0)
        break;
    end
    len=len+1;
end 
%%
s=serial('COM4');
fopen(s);
PI=3.14159265;
maxx=a(2);minx=0;maxy=b(2);miny=0;
   if(a(len-1)==a(len-2)&&b(len-1)==b(len-2))
        len=len-1;
   end 
   if((a(len)~=a(2))&&b(len)~=b(2))
   len=len+1 ;   
   a(len)=a(2);
   b(len)=b(2);
   end
    dx=maxx-minx;
    dy=maxy-miny;
    y=power(dy,2);
    x=power(dx,2);
	sum=x+y;
	dist=power(sum,0.5);
    tanangle=dy/dx;
    angle=atan(tanangle)*180/PI;
    
    angle=angle/10;
    angle=round(angle);
    angle=angle*10;
    ang=int2str(angle);
    ang=char(ang);
    la=length(ang);
    k=la-1;
     fprintf(s,'g');
     pause(2); 
     fprintf(s,'s');
     pause(1);
     fprintf(s,'h');
     pause(1);
     fprintf(s,'1');
     pause(5);
    while(k>0)
        if(k==la-1)
            data=ang(k);
            fprintf(s,'g');
            pause(2); 
            fprintf(s,'a');
            pause(1);
            fprintf(s,'t');
            pause(1);
            fprintf(s,data);
            pause(5);
        end
        
        if(k==la-2)
            data=ang(k);
            fprintf(s,'g');
            pause(2); 
            fprintf(s,'a');
            pause(1);
            fprintf(s,'h');
            pause(1);
            fprintf(s,data);
            pause(5);  
        end 
        k=k-1;
    end    
    fprintf('left = %f',angle);
    fprintf('\n');
    
    dist=dist/10;
    dist=round(dist);
    dist=dist*10;
    dis=int2str(dist);
    dis=char(dis);
    ld=length(dis);
    k=ld-1;
    while(k>0)
        if(k==ld-1)
            data=dis(k);
            fprintf(s,'g');
            pause(2); 
            fprintf(s,'w');
            pause(1);
            fprintf(s,'t');
            pause(1);
            fprintf(s,data);
            pause(5);
        end
        
        if(k==ld-2)
            data=dis(k);
            fprintf(s,'g');
            pause(2); 
            fprintf(s,'w');
            pause(1);
            fprintf(s,'h');
            pause(1);
            fprintf(s,data);
            pause(5);  
        end 
        k=k-1;
    end        
    fprintf('fwd = %f',round(dist)); 
    fprintf('\n');
    
    for i=2:len-1   
        minx=a(i);
        miny=b(i);
        maxx=a(i+1);
        maxy=b(i+1);
        dy=maxy-miny;
        dx=maxx-minx;
        l=distance(a(i),b(i),a(i-1),b(i-1));
        m=distance(a(i+1),b(i+1),a(i),b(i));
        n=distance(a(i+1),b(i+1),a(i-1),b(i-1));
        acosterm=(m*m+l*l-n*n)/(2*m*l);
        tempangle=acos(acosterm);
        tempangle=tempangle*180/PI;
        angle=180-tempangle;
        y=power(dy,2);
	    x=power(dx,2);
	    sum=x+y;
	    dist=power(sum,0.5);

        x1=a(i-1);y1=b(i-1);
        x2=a(i);y2=b(i);
        x3=a(i+1);y3=b(i+1);
	    dir=(x2-x1)*(y3-y1)-(x3-x1)*(y2-y1);
        
        angle=angle/10;
        angle=round(angle);
        angle=angle*10;
	    ang=int2str(angle);
        ang=char(ang);
        la=length(ang);
        k=la-1;
         fprintf(s,'g');
         pause(2); 
         fprintf(s,'s');
         pause(1);
         fprintf(s,'h');
         pause(1);
         fprintf(s,'1');
         pause(5);
        if(dir>0)
        while(k>0)
           if(k==la-1)
             data=ang(k);
             fprintf(s,'g');
             pause(2); 
             fprintf(s,'a');
             pause(1);
             fprintf(s,'t');
             pause(1);
             fprintf(s,data);
             pause(5);
           end
        
           if(k==la-2)
             data=ang(k);
             fprintf(s,'g');
             pause(2); 
             fprintf(s,'a');
             pause(1);
             fprintf(s,'h');
             pause(1);
             fprintf(s,data);
             pause(5);  
           end 
           k=k-1;
        end               
		   fprintf('left = %f',angle);
           fprintf('\n');
        end
        
        if(dir<0)
        while(k>0)
           if(k==la-1)
             data=ang(k); 
             fprintf(s,'g');
             pause(2); 
             fprintf(s,'d');
             pause(1);
             fprintf(s,'t');
             pause(1);
             fprintf(s,data);
             pause(5);
           end
        
           if(k==la-2)
             data=ang(k);  
             fprintf(s,'g');
             pause(2); 
             fprintf(s,'d');
             pause(1);
             fprintf(s,'h');
             pause(1);
             fprintf(s,data);
             pause(5);  
           end 
           k=k-1;
        end              
		  fprintf('right = %f',angle);
          fprintf('\n');
        end
         fprintf(s,'g');
         pause(2); 
         fprintf(s,'w');
         pause(1);
         fprintf(s,'t');
         pause(1);
         fprintf(s,'4');
         pause(5);
        
        dist=dist/10;
        dist=round(dist);
        dist=dist*10;
	    dis=int2str(dist);
        dis=char(dis);
        ld=length(dis);
        k=ld-1;
        fprintf(s,'[');
        pause(2); 
        while(k>0)
           if(k==ld-1)
             data=dis(k);  
             fprintf(s,'g');
             pause(2); 
             fprintf(s,'w');
             pause(1);
             fprintf(s,'t');
             pause(1);
             fprintf(s,data);
             pause(5);
           end
        
           if(k==ld-2)
             data=dis(k); 
             fprintf(s,'g');
             pause(2); 
             fprintf(s,'w');
             pause(1);
             fprintf(s,'h');
             pause(1);
             fprintf(s,data);
             pause(5);  
           end 
           k=k-1;
        end          
        fprintf('fwd = %f',dist);fprintf('\n'); 
         fprintf(s,']');
        pause(2); 
    end 
fclose(s);
delete(s);
%%
%%
%motion control blue
I=im_b;

BW=im_b;
[B,L,N] = bwboundaries(BW);

figure; imshow(BW); hold on;
for k=1:length(B),
    boundary = B{k};
     plot(boundary(:,2),...
       boundary(:,1),'b','LineWidth',2);
end
hold on
%% Constants
Window = 3;
Sigma = 2;
K = 0.05;
nCorners = 100;

%% Derivative masks
dx = [-1 0 1; -1 0 1; -1 0 1];
dy = dx';   %SO code color fix '

%% Find the image gradient
Mask = im2bw(I);
Ix = conv2(double(Mask),dx,'same');   
Iy = conv2(double(Mask),dy,'same');

%% Use a gaussian windowing function and compute the rest
Gaussian = fspecial('gaussian',Window,Sigma);
Ix2 = conv2(Ix.^2,  Gaussian, 'same');  
Iy2 = conv2(Iy.^2,  Gaussian, 'same');
Ixy = conv2(Ix.*Iy, Gaussian, 'same');    

%% Find the corners
CornerStrength = (Ix2.*Iy2 - Ixy.^2) - K*(Ix2 + Iy2).^2;
[val ind] = sort(CornerStrength(:),'descend');    
[Ci Cj] = ind2sub(size(CornerStrength),ind(1:nCorners));
x=size(boundary);
for i=1:nCorners
    for j=1:x(1,1)
        if((Ci(i,1)==boundary(j,1))&&((Cj(i,1)==boundary(j,2))))
            break;
        end 
        if(j==x(1,1))
            Ci(i,1)=0;
            Cj(i,1)=0;
        end        
    end    
end    
%%
c=zeros(nCorners,2);
c(1,1)=Cj(1,1);
c(1,2)=Ci(1,1);
err=0;
for i=2:nCorners
    for j=1:nCorners 
        temp1=abs(c(j,1)-Cj(i,1));
        temp2=abs(c(j,2)-Ci(i,1));
        if(temp1<10&&temp2<10)
            err=1;
        end    
    end
    if(err==0)
        c(i,1)=Cj(i,1);
        c(i,2)=Ci(i,1);
    end 
    err=0;
end
plot(c(:,1), c(:,2), 'r*');

%%
g=zeros(nCorners,2);
s=0;
for k=1:x(1,1)
    for l=1:nCorners
        if((boundary(k,1)==c(l,2))&&(boundary(k,2)==c(l,1)))
            s=s+1;
            g(s,2)=boundary(k,1);
            g(s,1)=boundary(k,2);
            break;
        end    
    end
end  
for i=2:nCorners
    if(g(i,1)==g(i-1,1)&&g(i,2)==g(i-1,2))
        g(i,1)=0;
        g(i,2)=0;
    end    
end  
%%
j=2;
a=zeros(nCorners,1);
for i=1:nCorners
    if(g(i,1)==0&&g(i,2)==0)
        continue;
    end 
    a(j,1)=g(i,2);
    j=j+1;
end 
j=2;
b=zeros(nCorners,1);
for i=1:nCorners
    if(g(i,1)==0&&g(i,2)==0)
        continue;
    end 
    b(j,1)=g(i,1);
    j=j+1;
end 
len=1;
for i=2:nCorners
    if(a(i,1)==0)
        break;
    end
    len=len+1;
end 
%%
s=serial('COM4');
fopen(s);
PI=3.14159265;
maxx=a(2);minx=0;maxy=b(2);miny=0;
   if(a(len-1)==a(len-2)&&b(len-1)==b(len-2))
        len=len-1;
   end     
   if((a(len)~=a(2))&&b(len)~=b(2))
   len=len+1 ;   
   a(len)=a(2);
   b(len)=b(2);
   end
    dx=maxx-minx;
    dy=maxy-miny;
    y=power(dy,2);
    x=power(dx,2);
	sum=x+y;
	dist=power(sum,0.5);
    tanangle=dy/dx;
    angle=atan(tanangle)*180/PI;
    
    angle=angle/10;
    angle=round(angle);
    angle=angle*10;
    ang=int2str(angle);
    ang=char(ang);
    la=length(ang);
    k=la-1;  
    fprintf(s,'b');
    pause(2); 
    fprintf(s,'s');
    pause(1);
    fprintf(s,'h');
    pause(1);
    fprintf(s,'1');
    pause(5);
    while(k>0)
        if(k==la-1)
            data=ang(k);
            fprintf(s,'b');
            pause(2); 
            fprintf(s,'a');
            pause(1);
            fprintf(s,'t');
            pause(1);
            fprintf(s,data);
            pause(5);
        end
        
        if(k==la-2)
            data=ang(k);
            fprintf(s,'b');
            pause(2); 
            fprintf(s,'a');
            pause(1);
            fprintf(s,'h');
            pause(1);
            fprintf(s,data);
            pause(5);  
        end 
        k=k-1;
    end    
    fprintf('left = %f',angle);
    fprintf('\n');
    fprintf(s,'b');
    pause(2); 
    fprintf(s,'w');
    pause(1);
    fprintf(s,'t');
    pause(1);
    fprintf(s,'4');
    pause(5);
    
    dist=dist/10;
    dist=round(dist);
    dist=dist*10;
    dis=int2str(dist);
    dis=char(dis);
    ld=length(dis);
    k=ld-1;
    while(k>0)
        if(k==ld-1)
            data=dis(k);
            fprintf(s,'b');
            pause(2); 
            fprintf(s,'w');
            pause(1);
            fprintf(s,'t');
            pause(1);
            fprintf(s,data);
            pause(5);
        end
        
        if(k==ld-2)
            data=dis(k);
            fprintf(s,'b');
            pause(2); 
            fprintf(s,'w');
            pause(1);
            fprintf(s,'h');
            pause(1);
            fprintf(s,data);
            pause(5);  
        end 
        k=k-1;
    end        
    fprintf('fwd = %f',round(dist)); 
    fprintf('\n');
    
    for i=2:len-1   
        minx=a(i);
        miny=b(i);
        maxx=a(i+1);
        maxy=b(i+1);
        dy=maxy-miny;
        dx=maxx-minx;
        l=distance(a(i),b(i),a(i-1),b(i-1));
        m=distance(a(i+1),b(i+1),a(i),b(i));
        n=distance(a(i+1),b(i+1),a(i-1),b(i-1));
        acosterm=(m*m+l*l-n*n)/(2*m*l);
        tempangle=acos(acosterm);
        tempangle=tempangle*180/PI;
        angle=180-tempangle;
        y=power(dy,2);
	    x=power(dx,2);
	    sum=x+y;
	    dist=power(sum,0.5);

        x1=a(i-1);y1=b(i-1);
        x2=a(i);y2=b(i);
        x3=a(i+1);y3=b(i+1);
	    dir=(x2-x1)*(y3-y1)-(x3-x1)*(y2-y1);
        
        angle=angle/10;
        angle=round(angle);
        angle=angle*10;
	    ang=int2str(angle);
        ang=char(ang);
        la=length(ang);
        k=la-1;
         fprintf(s,'b');
         pause(2); 
         fprintf(s,'s');
         pause(1);
         fprintf(s,'h');
         pause(1);
         fprintf(s,'1');
         pause(5);
        if(dir>0)
        while(k>0)
           if(k==la-1)
             data=ang(k);
             fprintf(s,'b');
             pause(2); 
             fprintf(s,'a');
             pause(1);
             fprintf(s,'t');
             pause(1);
             fprintf(s,data);
             pause(5);
           end
        
           if(k==la-2)
             data=ang(k);
             fprintf(s,'b');
             pause(2); 
             fprintf(s,'a');
             pause(1);
             fprintf(s,'h');
             pause(1);
             fprintf(s,data);
             pause(5);  
           end 
           k=k-1;
        end               
		   fprintf('left = %f',angle);
           fprintf('\n');
        end
        
        if(dir<0)
        while(k>0)
           if(k==la-1)
             data=ang(k);
             fprintf(s,'b');
             pause(2); 
             fprintf(s,'d');
             pause(1);
             fprintf(s,'t');
             pause(1);
             fprintf(s,data);
             pause(5);
           end
        
           if(k==la-2)
             data=ang(k); 
             fprintf(s,'b');
             pause(2); 
             fprintf(s,'d');
             pause(1);
             fprintf(s,'h');
             pause(1);
             fprintf(s,data);
             pause(5);  
           end 
           k=k-1;
        end              
		  fprintf('right = %f',angle);
          fprintf('\n');
        end
        fprintf(s,'b');
        pause(2); 
        fprintf(s,'w');
        pause(1);
        fprintf(s,'t');
        pause(1);
        fprintf(s,'4');
        pause(5);
        
        dist=dist/10;
        dist=round(dist);
        dist=dist*10;
	    dis=int2str(dist);
        dis=char(dis);
        ld=length(dis);
        k=ld-1;
        fprintf(s,'[');
        pause(2); 
        while(k>0)
           if(k==ld-1)
             data=dis(k); 
             fprintf(s,'b');
             pause(2); 
             fprintf(s,'w');
             pause(1);
             fprintf(s,'t');
             pause(1);
             fprintf(s,data);
             pause(5);
           end
        
           if(k==ld-2)
             data=dis(k); 
             fprintf(s,'b');
             pause(2); 
             fprintf(s,'w');
             pause(1);
             fprintf(s,'h');
             pause(1);
             fprintf(s,data);
             pause(5);  
           end 
           k=k-1;
        end          
        fprintf('fwd = %f',dist);fprintf('\n'); 
         fprintf(s,']');
        pause(2); 
    end 
fprintf(s,'<');    
fclose(s);
delete(s);





