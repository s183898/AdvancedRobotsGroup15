addpath([pwd filesep 'extras'])
addpath([pwd filesep 'yourScripts'])
constants

wl=[0.71,1]
 pose=[0.1 0.2 0.3]
 cov=eye(3)
[line,covo]= projectToLaser(wl,pose,cov);
line 
linecor = [0.4100    0.5370]
covo

covocor= [1.0000    0.1116
          0.1116    1.0125]