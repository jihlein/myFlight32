clc
clear all

setupBuses;

load telem;

simTime = length(inputData(:,1)) * 0.01;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

commandDT = 1 / 50;
filterDT  = 1 / 100;
%controlDT = 1 / 100;
controlDT = 1 / 500;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

eepromConfig.version                        = uint8(0);

eepromConfig.mpuTempMin                     = single(0);

eepromConfig.mpuTempMax                     = single(0);

eepromConfig.accelBiasPolynomial            = zeros(3,5,'single');

eepromConfig.accelScaleFactorPolynomial     = zeros(3,5,'single');

eepromConfig.gyroBiasPolynomial             = zeros(3,5,'single');
    
eepromConfig.magBias                        = zeros(1,6,'single');
    
eepromConfig.accelCutoff                    = single(0.25);
    
eepromConfig.kpAcc                          = single(1);
    
eepromConfig.kpMag                          = single(5);
    
eepromConfig.earthAccel100HzHPtau           = single(4);

eepromConfig.posEstA                        = single(2);
    
eepromConfig.posEstB                        = single(1);
    
eepromConfig.hEstA                          = single(2);
    
eepromConfig.hEstB                          = single(1);
    
eepromConfig.dlpfSetting                    = uint8(0);
    
eepromConfig.sensorOrientation              = uint8(0);

eepromConfig.attTrim                        = zeros(1,2,'single');

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    
eepromConfig.rollAndPitchRateScaling        = single(0);
eepromConfig.yawRateScaling                 = single(0);
    
eepromConfig.attitudeScaling                = single(0);
    
eepromConfig.nDotEdotScaling                = single(0);
    
eepromConfig.hDotScaling                    = single(0);
    
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    
eepromConfig.receiverType                   = uint8(0);
    
eepromConfig.slaveSpektrum                  = uint8(0);
    
eepromConfig.rcMap                          = zeros(1,8,'uint8');
    
eepromConfig.escPwmRate                     = uint16(450);
eepromConfig.servoPwmRate                   = uint16(50);
    
eepromConfig.midCommand                     = single(0);
eepromConfig.minCheck                       = single(0);
eepromConfig.maxCheck                       = single(0);
eepromConfig.minThrottle                    = single(0);
eepromConfig.maxThrottle                    = single(0);
    
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    
eepromConfig.mixerConfiguration             = uint8(0);
eepromConfig.yawDirection                   = single(1);
    
eepromConfig.triYawServoPwmRate             = uint16(50);
eepromConfig.triYawServoMin                 = single(2000);
eepromConfig.triYawServoMid                 = single(3000);
eepromConfig.triYawServoMax                 = single(4000);
eepromConfig.triCopterYawCmd500HzLowPassTau = single(0.05);
    
eepromConfig.freeMixMotors                  = uint8(4);

eepromConfig.freeMix                        = zeros(8,3,'single');
    
eepromConfig.freeMix(1,1)                   = single( 1);
eepromConfig.freeMix(1,2)                   = single(-1);
eepromConfig.freeMix(1,3)                   = single(-1);
eepromConfig.freeMix(1,4)                   = single( 1);
  
eepromConfig.freeMix(2,1)                   = single(-1);
eepromConfig.freeMix(2,2)                   = single(-1);
eepromConfig.freeMix(2,3)                   = single( 1);
eepromConfig.freeMix(2,4)                   = single( 1);

eepromConfig.freeMix(3,1)                   = single(-1);
eepromConfig.freeMix(3,2)                   = single( 1);
eepromConfig.freeMix(3,3)                   = single(-1);
eepromConfig.freeMix(3,4)                   = single( 1);

eepromConfig.freeMix(4,1)                   = single( 1);
eepromConfig.freeMix(4,2)                   = single( 1);
eepromConfig.freeMix(4,3)                   = single( 1);
eepromConfig.freeMix(4,4)                   = single( 1);

eepromConfig.freeMix(5,1)                   = single( 0);
eepromConfig.freeMix(5,2)                   = single( 0);
eepromConfig.freeMix(5,3)                   = single( 0);
eepromConfig.freeMix(5,4)                   = single( 0);

eepromConfig.freeMix(6,1)                   = single( 0);
eepromConfig.freeMix(6,2)                   = single( 0);
eepromConfig.freeMix(6,3)                   = single( 0);
eepromConfig.freeMix(6,4)                   = single( 0);
    
eepromConfig.freeMix(7,1)                   = single( 0);
eepromConfig.freeMix(7,2)                   = single( 0);
eepromConfig.freeMix(7,3)                   = single( 0);
eepromConfig.freeMix(7,4)                   = single( 0);
    
eepromConfig.freeMix(8,1)                   = single( 0);
eepromConfig.freeMix(8,2)                   = single( 0);
eepromConfig.freeMix(8,3)                   = single( 0);
eepromConfig.freeMix(8,4)                   = single( 0);
    
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    
eepromConfig.rateP                          = single([250.0 250.0 350.0]);
eepromConfig.rateI                          = single([100.0 100.0 100.0]);
eepromConfig.rateD                          = single([  0.0   0.0   0.0]);
eepromConfig.rateLimit                      = single([ 500.0  500.0  500.0]);

eepromConfig.attitudeP                      = single([  2.0   2.0   3.0]);
eepromConfig.attitudeI                      = single([  0.0   0.0   0.0]);
eepromConfig.attitudeD                      = single([  0.0   0.0   0.0]);
eepromConfig.attitudeLimit                  = single([ 500.0  500.0  500.0]);

eepromConfig.velocityP                      = single([250.0 250.0 350.0]);
eepromConfig.velocityI                      = single([100.0 100.0 100.0]);
eepromConfig.velocityD                      = single([  0.0   0.0   0.0]);
eepromConfig.velocityLimit                  = single([ 500.0  500.0  500.0]);

eepromConfig.positionP                      = single([  2.0   2.0   3.0]);
eepromConfig.positionI                      = single([  0.0   0.0   0.0]);
eepromConfig.positionD                      = single([  0.0   0.0   0.0]);
eepromConfig.positionLimit                  = single([ 500.0  500.0  500.0]);

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    
eepromConfig.rollAttAltCompensationGain     = single(1);
eepromConfig.rollAttAltCompensationLimit    = single(0);
    
eepromConfig.pitchAttAltCompensationGain    = single(1);
eepromConfig.pitchAttAltCompensationLimit   = single(0);
    
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    
eepromConfig.batteryCells                   = uint8(3);
eepromConfig.voltageMonitorScale            = single(0);
eepromConfig.voltageMonitorBias             = single(0);
    
eepromConfig.batteryLow                     = single(0);
eepromConfig.batteryVeryLow                 = single(0);
eepromConfig.batteryMaxLow                  = single(0);
    
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    
eepromConfig.armCount                       = uint8(50);
eepromConfig.disarmCount                    = uint8(0);
    
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    
eepromConfig.activeTelemetry                = uint16(0);
    
eepromConfig.mavlinkEnabled                 = uint8(0);
    
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    
eepromConfig.useGPS                         = uint8(0);

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

eepromConfig.verticalVelocityHoldOnly       = uint8(1);
    
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    
eepromConfig.externalHMC5883                = uint8(0);
eepromConfig.externalMS5611                 = uint8(0);
    
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    
eepromConfig.CRCFlags                       = uint8(0);
eepromConfig.CRCAtEnd                       = uint32(0);
    
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

p = Simulink.Parameter;

p.Value = eepromConfig;
p.CoderInfo.StorageClass = 'ImportedExtern';
p.DataType = 'Bus: eepromConfig_t';

eepromConfig = p;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

accelConfidenceDecay = 1 / sqrt(eepromConfig.Value.accelCutoff);

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% TAU = Filter Time Constant
% T   = Filter Sample Time

% A   = 2 * TAU / T

% Low Pass:
% GX1 = 1 / (1 + A)
% GX2 = 1 / (1 + A)
% GX3 = (1 - A) / (1 + A)

% High Pass:
% GX1 =  A / (1 + A)
% GX2 = -A / (1 + A)
% GX3 = (1 - A) / (1 + A)

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

a = single(2 * eepromConfig.Value.earthAccel100HzHPtau / filterDT);

gx1 =  a / (1 + a);
gx2 = -a / (1 + a);
gx3 =  (1 - a) / (1 + a);

earthAccelNumHP = [gx1 gx2];
earthAccelDenHP = [1   gx3];

p = Simulink.Parameter;

p.Value = earthAccelNumHP;
p.CoderInfo.StorageClass = 'ExportedGlobal';
p.DataType = 'single';

earthAccelNumHP = p;

p = Simulink.Parameter;

p.Value = earthAccelDenHP;
p.CoderInfo.StorageClass = 'ExportedGlobal';
p.DataType = 'single';

earthAccelDenHP = p;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

a = single(2 * eepromConfig.Value.triCopterYawCmd500HzLowPassTau / commandDT);

gx1 = 1 / (1 + a);
gx2 = 1 / (1 + a);
gx3 = (1 - a) / (1 + a);

triYawRateNumLP = [gx1 gx2];
triYawRateDenLP = [1   gx3];

p = Simulink.Parameter;

p.Value = triYawRateNumLP;
p.CoderInfo.StorageClass = 'ExportedGlobal';
p.DataType = 'single';

triYawRateNumLP = p;

p = Simulink.Parameter;

p.Value = triYawRateDenLP;
p.CoderInfo.StorageClass = 'ExportedGlobal';
p.DataType = 'single';

triYawRateDenLP = p;

clear a gx1 gx2 gx3

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

numMotors = 8;

mixTri(1,1) = single( 1);
mixTri(1,2) = single(-0.666667);
mixTri(1,3) = single( 0);
mixTri(1,4) = single( 1);
  
mixTri(2,1) = single(-1);
mixTri(2,2) = single(-0.666667);
mixTri(2,3) = single( 0);
mixTri(2,4) = single( 1);

mixTri(3,1) = single( 0);
mixTri(3,2) = single( 1.333333);
mixTri(3,3) = single( 0);
mixTri(3,4) = single( 1);

mixTri(4,1) = single( 0);
mixTri(4,2) = single( 0);
mixTri(4,3) = single( 0);
mixTri(4,4) = single( 0);

mixTri(5,1) = single( 0);
mixTri(5,2) = single( 0);
mixTri(5,3) = single( 0);
mixTri(5,4) = single( 0);

mixTri(6,1) = single( 0);
mixTri(6,2) = single( 0);
mixTri(6,3) = single( 0);
mixTri(6,4) = single( 0);

mixTri(7,1) = single( 0);
mixTri(7,2) = single( 0);
mixTri(7,3) = single( 0);
mixTri(7,4) = single( 0);

mixTri(8,1) = single( 0);
mixTri(8,2) = single( 0);
mixTri(8,3) = single( 0);
mixTri(8,4) = single( 0);

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

mixQuadX(1,1) = single( 1);
mixQuadX(1,2) = single(-1);
mixQuadX(1,3) = single(-1);
mixQuadX(1,4) = single( 1);
  
mixQuadX(2,1) = single(-1);
mixQuadX(2,2) = single(-1);
mixQuadX(2,3) = single( 1);
mixQuadX(2,4) = single( 1);

mixQuadX(3,1) = single(-1);
mixQuadX(3,2) = single( 1);
mixQuadX(3,3) = single(-1);
mixQuadX(3,4) = single( 1);

mixQuadX(4,1) = single( 1);
mixQuadX(4,2) = single( 1);
mixQuadX(4,3) = single( 1);
mixQuadX(4,4) = single( 1);

mixQuadX(5,1) = single( 0);
mixQuadX(5,2) = single( 0);
mixQuadX(5,3) = single( 0);
mixQuadX(5,4) = single( 0);

mixQuadX(6,1) = single( 0);
mixQuadX(6,2) = single( 0);
mixQuadX(6,3) = single( 0);
mixQuadX(6,4) = single( 0);

mixQuadX(7,1) = single( 0);
mixQuadX(7,2) = single( 0);
mixQuadX(7,3) = single( 0);
mixQuadX(7,4) = single( 0);

mixQuadX(8,1) = single( 0);
mixQuadX(8,2) = single( 0);
mixQuadX(8,3) = single( 0);
mixQuadX(8,4) = single( 0);

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

mixHexX(1,1) = single( 0.866025);
mixHexX(1,2) = single(-1);
mixHexX(1,3) = single(-1);
mixHexX(1,4) = single( 1);
  
mixHexX(2,1) = single(-0.866025);
mixHexX(2,2) = single(-1);
mixHexX(2,3) = single( 1);
mixHexX(2,4) = single( 1);

mixHexX(3,1) = single(-0.866025);
mixHexX(3,2) = single( 0);
mixHexX(3,3) = single(-1);
mixHexX(3,4) = single( 1);

mixHexX(4,1) = single(-0.866025);
mixHexX(4,2) = single( 1);
mixHexX(4,3) = single( 1);
mixHexX(4,4) = single( 1);

mixHexX(5,1) = single( 0.866025);
mixHexX(5,2) = single( 1);
mixHexX(5,3) = single(-1);
mixHexX(5,4) = single( 1);

mixHexX(6,1) = single( 0.866025);
mixHexX(6,2) = single( 0);
mixHexX(6,3) = single( 1);
mixHexX(6,4) = single( 1);

mixHexX(7,1) = single( 0);
mixHexX(7,2) = single( 0);
mixHexX(7,3) = single( 0);
mixHexX(7,4) = single( 0);

mixHexX(8,1) = single( 0);
mixHexX(8,2) = single( 0);
mixHexX(8,3) = single( 0);
mixHexX(8,4) = single( 0);

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

mixY6(1,1) = single( 1);
mixY6(1,2) = single(-0.666667);
mixY6(1,3) = single(-1);
mixY6(1,4) = single( 1);
  
mixY6(2,1) = single(-1);
mixY6(2,2) = single(-0.666667);
mixY6(2,3) = single( 1);
mixY6(2,4) = single( 1);

mixY6(3,1) = single( 0);
mixY6(3,2) = single( 1.333333);
mixY6(3,3) = single( 1);
mixY6(3,4) = single( 1);

mixY6(4,1) = single( 1);
mixY6(4,2) = single(-0.666667);
mixY6(4,3) = single( 1);
mixY6(4,4) = single( 1);

mixY6(5,1) = single(-1);
mixY6(5,2) = single(-0.666667);
mixY6(5,3) = single(-1);
mixY6(5,4) = single( 1);

mixY6(6,1) = single( 0);
mixY6(6,2) = single( 1.333333);
mixY6(6,3) = single(-1);
mixY6(6,4) = single( 1);

mixY6(7,1) = single( 0);
mixY6(7,2) = single( 0);
mixY6(7,3) = single( 0);
mixY6(7,4) = single( 0);

mixY6(8,1) = single( 0);
mixY6(8,2) = single( 0);
mixY6(8,3) = single( 0);
mixY6(8,4) = single( 0);

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%