addpath([pwd filesep 'extras'])
addpath([pwd filesep 'yourScripts'])
constants

pose=[0.5;0.5;pi]

poseCov=[1.0620349e-03  -4.8430253e-05  -1.3467830e-03
  -4.8430253e-05   2.8061827e-04   5.8793326e-05
  -1.3467830e-03   5.8793326e-05   3.0235923e-03]

matchResult = [     0   -1.5708   -3.1416    1.5708
                    0.9200    0.9200    0.9200    0.9200
                       0    0.0388    0.0388    0.0390
                       0    0.0065   -0.0132   -0.0064
                       0    3.0000    2.0000    1.0000]

[pose, poseCov] = measurementUpdate(pose,poseCov, matchResult);

pose

posecor =[0.4993; 0.4988; 3.111]

poseCov

poseCovcor = 1.0e-03 * [0.2264   -0.0138   -0.0572
                       -0.0138    0.1239    0.0448
                       -0.0572    0.0448    0.2692]