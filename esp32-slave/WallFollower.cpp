/**
 * WallFollower.cpp
 * Source for wall follower class. 
 * 
 * Grand Theft Autonomous -- Group 28
 * Mechatronics (MEAM 510)
 * 
 * @author Ethan J. Musser
 * @version 0.1
 */

#include "WallFollower.h"

WallFollower::WallFollower(MecanumBase& base,
                           TripleTOF& ttof,
                           unsigned int rightSensorDistance,
                           unsigned int setDistance,
                           unsigned int turnDistance,
                           unsigned int rotDistance,
                           float kpWall,
                           float kpRotation,
                           float findSpeed,
                           float followSpeed,
                           float rotationSpeed)
    : _base(&base),
      _ttof(&ttof),
      _sensorDistance(rightSensorDistance),
      _setDistance(setDistance),
      _rotDistance(rotDistance),
      _turnDistance(turnDistance),
      _gains{kpWall, kpRotation},
      _findSpeed(findSpeed),
      _followSpeed(followSpeed),
      _rotationSpeed(rotationSpeed)
{}

void WallFollower::enable()
{
    _state = findingWall;
}

void WallFollower::disable()
{
    _state = notFollowing;
    _base->stop();
}

void WallFollower::update()
{
    if(_state != notFollowing) {
        _currentPose = parseRanges(_ttof->getRanges());
        switch(_state) {
            case findingWall:
                findWall();
                break;
            case wallFollowing:
                followWall();
                break;
            case turningCorner:
                turnCorner();
                break;
        }
    }
}

void WallFollower::findWall()
{
    if(_currentPose.right <= _setDistance) {
        _state = wallFollowing;
    } else {
        _base->driveCartesian(0.0, _findSpeed, 0.0);
    }
}

void WallFollower::followWall()
{
    if(_currentPose.front <= _turnDistance) {
        _state = turningCorner;
    } else {
        Twist2D control = wallControl(_currentPose, _setDistance, _followSpeed, 
                _gains);
    }
}

void WallFollower::turnCorner()
{
    if(_currentPose.front >= _rotDistance) {
        _state = findingWall;
    } else {
        _base->driveCartesian(0.0, 0.0, _rotationSpeed);  
    }
}

Twist2D WallFollower::wallControl(WallPose pose, 
                                  unsigned int setDistance,
                                  float followSpeed,
                                  FollowerGains gains)
{
    Twist2D control;
    control.vx = followSpeed;
    control.vy = gains.kpWall * ((float) (pose.right - (int) setDistance)) / 1.0e3;
    control.vtheta = gains.kpRot * pose.theta;
    return control;
}

WallPose WallFollower::parseRanges(TripleRange ranges)
{
    WallPose pose;
    pose.front = ranges.r1;
    pose.right = 0.5 * (ranges.r2 + ranges.r3);
    pose.theta = atan2((float) (ranges.r3 - ranges.r2), (float) _sensorDistance);
    return pose;
}

