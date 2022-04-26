addpath([pwd filesep 'extras'])
addpath([pwd filesep 'yourScripts'])
constants

pose=[5.0000000e-01
   5.0000000e-01
   3.1415927e+00]
poseCov=[1.0620349e-03  -4.8430253e-05  -1.3467830e-03
  -4.8430253e-05   2.8061827e-04   5.8793326e-05
  -1.3467830e-03   5.8793326e-05   3.0235923e-03]

worldLines=[0.0000000e+00  -1.5707963e+00  -3.1415927e+00   1.5707963e+00
            9.2000000e-01   9.2000000e-01   9.2000000e-01   9.2000000e-01]
for i=1:4 
tmp=projectToLaser( worldLines(:,i),pose, poseCov);
predictedlines(:,i)=tmp';
end
predictedlines
 

laserLines=[
  -1.5318372e+00   3.8827152e-02   1.6095624e+00
   4.1355322e-01   1.1268265e+00   1.4264938e+00]

matchResult = match(pose, poseCov,worldLines, laserLines)

matchResultcor = [     0   -1.5708   -3.1416    1.5708
                    0.9200    0.9200    0.9200    0.9200
                       0    0.0388    0.0388    0.0390
                       0    0.0065   -0.0132   -0.0064
                       0    3.0000    2.0000    1.0000]

