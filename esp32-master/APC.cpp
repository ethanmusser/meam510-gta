/**
 * APC.cpp
 * Source for absolute position control (APC) class. 
 * 
 * Grand Theft Autonomous -- Group 28
 * Mechatronics (MEAM 510)
 * 
 * @author Ethan J. Musser
 * @version 0.1
 */

#include "APC.h"
#include <math.h>

APC::APC(MecanumBase& base,
         Vive510& vive,
         float xOffset,
         float yOffset,
         float kpTrans,
         float kdTrans,
         float epsilonT)
    : _base(&base),
      _frontVive(&vive),
      _offsets{xOffset, yOffset, 0.0},
      _transGains{kpTrans, kdTrans},
      _rotGains{0.0, 0.0},
      _epsilons{epsilonT, 0.0},
      _hasRearVive(false)
{}

APC::APC(MecanumBase& base,
         Vive510& frontVive, 
         Vive510& rearVive,
         float xOffset,
         float yOffset,
         float qOffset,
         float kpTrans,
         float kdTrans,
         float kpRot,
         float kdRot,
         float epsilonT,
         float epsilonR)
    : _base(&base),
      _frontVive(&frontVive),
      _rearVive(&rearVive),
      _offsets{xOffset, yOffset, 0.0},
      _transGains{kpTrans, kdTrans},
      _rotGains{kpRot, kdRot},
      _epsilons{epsilonT, epsilonR},
      _hasRearVive(true)
{}

void APC::setDestination(Pose2D pose)
{
    _desiredPose = pose;
}

void APC::setDestination(float x,
                         float y,
                         float theta)
{
    _desiredPose.x = x;
    _desiredPose.y = y;
    _desiredPose.theta = theta;
}

void APC::setOffsets(float xOffset,
                     float yOffset,
                     float qOffset)
{
    _offsets.x = xOffset;
    _offsets.y = yOffset;
    _offsets.theta = qOffset;
}

void APC::setGains(float kpTrans,
                   float kdTrans,
                   float kpRot,
                   float kdRot)
{
    setTranslationalGains(kpTrans, kdTrans);
    setRotationalGains(kpRot, kdRot);
}

void APC::setTranslationalGains(float kp,
                                float kd)
{
    _transGains.kp = kp;
    _transGains.kd = kd;
}

void APC::setRotationalGains(float kp,
                             float kd)
{
    _rotGains.kp = kp;
    _rotGains.kd = kd;
}

void APC::setEpsilons(float epsilonT,
                      float epsilonR)
{
    _epsilons.epsilonT = epsilonT;
    _epsilons.epsilonR = epsilonR;
}

void APC::update()
{
    // Compute Current Pose 
    computePose();
    Pose2D error = computeError(_currentPose, _desiredPose);
    float transNormError = computeTranslationalNorm(error);

    // Fix Desired Angle
    _desiredPose.theta = wrapAngle(atan2(error.y, error.x));
    error.theta = -_currentPose.theta + _desiredPose.theta;

    // Control Inputs
    if(_transControlEnabled || _rotControlEnabled){
        if(transNormError <= _epsilons.epsilonT) {
            disable();
        } else {
            Twist2D control = computeBaseControl(_currentPose, _currentTwist, 
                    _desiredPose, _transGains, _rotGains);
            // if(error.theta > _epsilons.epsilonR) {
            //     control.vx = 0.0;
            //     control.vy = 0.0;
            // } else if(transNormError > _epsilons.epsilonT) {
            //     control.vtheta = 0.0;
            // } 
            _base->driveCartesian(control.vx, control.vy, control.vtheta);
        }
    }
}

void APC::enable()
{
    enableTranslation();
    enableRotation();
}

void APC::enableTranslation()
{
    _transControlEnabled = true;
}

void APC::enableRotation()
{
    _rotControlEnabled = true;
}

void APC::disable()
{
    disableTranslation();
    disableRotation();
    _base->stop();
}

void APC::disableTranslation()
{
    _transControlEnabled = false;
}

void APC::disableRotation()
{
    _rotControlEnabled = false;
}

Pose2D APC::getPose() 
{
    if(millis() - _previousUpdateTime > 50) {
        computePose();
    }
    return _currentPose;
}

Twist2D APC::getTwist() 
{
    if(millis() - _previousUpdateTime > 50) {
        computePose();
    }
    return _currentTwist;
}

void APC::computePose()
{
    // Get Vive Pose(s)
    if(_frontVive->status() == VIVE_LOCKEDON) {
        _frontVivePose.x = ((float) _frontVive->xCoord()) / 1.0e3;
        _frontVivePose.y = ((float) _frontVive->yCoord()) / 1.0e3;
    } else {
        _frontVive->sync(5);
    }
    if(_hasRearVive) {
        if(_rearVive->status() == VIVE_LOCKEDON) {
            _rearVivePose.x = ((float) _rearVive->xCoord()) / 1.0e3;
            _rearVivePose.y = ((float) _rearVive->yCoord()) / 1.0e3;
        } else {
            _rearVive->sync(5);
        }
    }
    _previousPose = _currentPose;

    // Compute Base Pose
    if(!_hasRearVive) {
        _currentPose.x = _frontVivePose.x - _offsets.x;
        _currentPose.y = _frontVivePose.y - _offsets.y;
    } else {
        // _currentPose.x = _frontVivePose.x - _offsets.x;
        // _currentPose.y = _frontVivePose.y - _offsets.y;
        _currentPose.x = 0.5 * (_frontVivePose.x + _rearVivePose.x) - _offsets.x;
        _currentPose.y = 0.5 * (_frontVivePose.y + _rearVivePose.y) - _offsets.y;
        Pose2D viveDiff = computeError(_frontVivePose, _rearVivePose);
        float angle = atan2(viveDiff.y, viveDiff.x);
        _currentPose.theta = wrapAngle(angle);
    }

    // Compute Base Twist
    long currentTime = millis();
    float dt = (float) (currentTime - _previousUpdateTime);
    _previousUpdateTime = currentTime;
    _currentTwist.vx = (_currentPose.x - _previousPose.x) / dt;
    _currentTwist.vy = (_currentPose.y - _previousPose.y) / dt;
    if(_hasRearVive) {
        _currentTwist.vtheta = (_currentPose.theta - _previousPose.theta) / dt;
    }
}

Twist2D APC::computeBaseControl(Pose2D currentPose,
                                Twist2D currentTwist,
                                Pose2D desiredPose,
                                Gains transGains,
                                Gains rotGains)
{
    Twist2D control;
    if(_transControlEnabled) {
        control.vx = -computeControl(currentPose.x, currentTwist.vx, 
                desiredPose.x, transGains);
        control.vy = -computeControl(currentPose.y, currentTwist.vy, 
                desiredPose.y, transGains);
    }
    if(_hasRearVive && _rotControlEnabled) {
        control.vtheta = -computeControl(currentPose.theta, currentTwist.vtheta, 
                desiredPose.theta, rotGains);
    }
    return control;
}

float APC::computeControl(float currentPosition,
                          float currentVelocity,
                          float desiredPosition,
                          Gains gains)
{
    float error = currentPosition - desiredPosition;
    return -gains.kp * error - gains.kd * currentVelocity;
}

Pose2D APC::computeError(Pose2D current,
                         Pose2D desired)
{
    // Constrain Inputs
    current.theta = wrapAngle(current.theta);
    desired.theta = wrapAngle(desired.theta);

    // Compute Error
    Pose2D error;
    error.x = current.x - desired.x;
    error.y = current.y - desired.y;
    error.theta = current.theta - desired.theta;
    return error;
}

float APC::computeTotalNorm(Pose2D pose)
{
    return sqrt(pow(pose.x, 2) + pow(pose.y, 2) + pow(pose.theta, 2));
}

float APC::computeTranslationalNorm(Pose2D pose)
{
    return sqrt(pow(pose.x, 2) + pow(pose.y, 2));
}

float APC::wrapAngle(float angle,
                     float lower,
                     float upper)
{
    float range = upper - lower; // https://stackoverflow.com/questions/11498169/dealing-with-angle-wrap-in-c-code
    while(angle <= lower || angle > upper) {
        if(angle <= lower) angle += range;
        if(angle > upper) angle -= range;
    }
    return angle;
}

