#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <float.h>
#include <chrono>
#include "motion.h"
#include "interpolator.h"
#include "types.h"
#include <iostream>

Interpolator::Interpolator()
{
  //Set default interpolation type
  m_InterpolationType = LINEAR;

  //set default angle representation to use for interpolation
  m_AngleRepresentation = EULER;
}

Interpolator::~Interpolator()
{
}

//helper functions

template<typename T> T Interpolator::Lerp(T& start, T& end, double t)
{
    return start + (end - start) * t;
}

//Create interpolated motion
void Interpolator::Interpolate(Motion * pInputMotion, Motion ** pOutputMotion, int N) 
{
  //Allocate new motion
  *pOutputMotion = new Motion(pInputMotion->GetNumFrames(), pInputMotion->GetSkeleton()); 
  auto start = std::chrono::high_resolution_clock::now();
  //Perform the interpolation
  if ((m_InterpolationType == LINEAR) && (m_AngleRepresentation == EULER))
    LinearInterpolationEuler(pInputMotion, *pOutputMotion, N);
  else if ((m_InterpolationType == LINEAR) && (m_AngleRepresentation == QUATERNION))
    LinearInterpolationQuaternion(pInputMotion, *pOutputMotion, N);
  else if ((m_InterpolationType == BEZIER) && (m_AngleRepresentation == EULER))
    BezierInterpolationEuler(pInputMotion, *pOutputMotion, N);
  else if ((m_InterpolationType == BEZIER) && (m_AngleRepresentation == QUATERNION))
    BezierInterpolationQuaternion(pInputMotion, *pOutputMotion, N);
  else
  {
    printf("Error: unknown interpolation / angle representation type.\n");
    exit(1);
  }
  auto stop = std::chrono::high_resolution_clock::now();
  auto duration = std::chrono::duration_cast<std::chrono::microseconds>(stop - start);
  std::cout << "Time taken: " << duration.count() << " microseconds" << std::endl;
}

void Interpolator::LinearInterpolationEuler(Motion * pInputMotion, Motion * pOutputMotion, int N)
{
  int inputLength = pInputMotion->GetNumFrames(); // frames are indexed 0, ..., inputLength-1

  int startKeyframe = 0;
  while (startKeyframe + N + 1 < inputLength)
  {
    int endKeyframe = startKeyframe + N + 1;

    Posture * startPosture = pInputMotion->GetPosture(startKeyframe);
    Posture * endPosture = pInputMotion->GetPosture(endKeyframe);

    // copy start and end keyframe
    pOutputMotion->SetPosture(startKeyframe, *startPosture);
    pOutputMotion->SetPosture(endKeyframe, *endPosture);

    // interpolate in between
    for(int frame=1; frame<=N; frame++)
    {
      Posture interpolatedPosture;
      double t = 1.0 * frame / (N+1);

      // interpolate root position
      interpolatedPosture.root_pos = startPosture->root_pos * (1-t) + endPosture->root_pos * t;

      // interpolate bone rotations
      for (int bone = 0; bone < MAX_BONES_IN_ASF_FILE; bone++)
        interpolatedPosture.bone_rotation[bone] = startPosture->bone_rotation[bone] * (1-t) + endPosture->bone_rotation[bone] * t;

      pOutputMotion->SetPosture(startKeyframe + frame, interpolatedPosture);
    }

    startKeyframe = endKeyframe;
  }

  for(int frame=startKeyframe+1; frame<inputLength; frame++)
    pOutputMotion->SetPosture(frame, *(pInputMotion->GetPosture(frame)));
}

void Interpolator::Rotation2Euler(double R[9], double angles[3])
{
  double cy = sqrt(R[0]*R[0] + R[3]*R[3]);

  if (cy > 16*DBL_EPSILON) 
  {
    angles[0] = atan2(R[7], R[8]);
    angles[1] = atan2(-R[6], cy);
    angles[2] = atan2(R[3], R[0]);
  } 
  else 
  {
    angles[0] = atan2(-R[5], R[4]);
    angles[1] = atan2(-R[6], cy);
    angles[2] = 0;
  }

  for(int i=0; i<3; i++)
    angles[i] *= 180 / M_PI;
}


void Interpolator::Euler2Rotation(double angles[3], double R[9])
{
  double rotMatX[4][4], rotMatY[4][4], rotMatZ[4][4];
  rotationX(rotMatX, angles[0]);
  rotationY(rotMatY, angles[1]);
  rotationZ(rotMatZ, angles[2]);

  double temp[4][4], rotMatXYZ[4][4];
  matrix_mult(rotMatY, rotMatX, temp);
  matrix_mult(rotMatZ, temp, rotMatXYZ);

  for (int i = 0; i < 3; i++)
  {
      for (int j = 0; j < 3; j++)
      {
          R[3 * i + j] = rotMatXYZ[i][j];
      }
  }
}

void Interpolator::BezierInterpolationEuler(Motion * pInputMotion, Motion * pOutputMotion, int N)
{
    int inputLength = pInputMotion->GetNumFrames(); // frames are indexed 0, ..., inputLength-1

    int startKeyframe = 0;
    while (startKeyframe + N + 1 < inputLength)
    {
        int endKeyframe = startKeyframe + N + 1;

        Posture* startPosture = pInputMotion->GetPosture(startKeyframe);
        Posture* endPosture = pInputMotion->GetPosture(endKeyframe);

        // copy start and end keyframe
        pOutputMotion->SetPosture(startKeyframe, *startPosture);
        pOutputMotion->SetPosture(endKeyframe, *endPosture);

        // interpolate in between
        for (int frame = 1; frame <= N; frame++)
        {
            Posture interpolatedPosture;
            double t = 1.0 * frame / (N + 1);

            // interpolate root position
            //interpolatedPosture.root_pos = startPosture->root_pos * (1 - t) + endPosture->root_pos * t;
            vector pStartRoot, pEndRoot, aRoot, bRoot;
            pStartRoot = startPosture->root_pos;
            pEndRoot = endPosture->root_pos;
            // detect edge case (start frame is 0, or end frame is N)
            if (startKeyframe == 0)
            {
                vector pEndNextRoot = pInputMotion->GetPosture(endKeyframe + N + 1)->root_pos;
                vector temp = Lerp(pEndNextRoot, pEndRoot, 2.0);
                aRoot = Lerp(pStartRoot, temp, 1.0 / 3);
            }
            else
            {
                vector pStartPreviousRoot = pInputMotion->GetPosture(startKeyframe - N - 1)->root_pos;
                vector temp = Lerp(pStartPreviousRoot, pStartRoot, 2.0);
                temp = Lerp(temp, pEndRoot, 0.5);
                aRoot = Lerp(pStartRoot, temp, 1.0 / 3);
            }

            if (endKeyframe + N + 1 > inputLength)
            {
                vector pStartPreviousRoot = pInputMotion->GetPosture(startKeyframe - N - 1)->root_pos;
                vector temp = Lerp(pStartPreviousRoot, pStartRoot, 2.0);
                bRoot = Lerp(pEndRoot, temp, 1.0 / 3);
            }
            else
            {
                vector pEndNextRoot = pInputMotion->GetPosture(endKeyframe + N + 1)->root_pos;
                vector temp = Lerp(pStartRoot, pEndRoot, 2.0);
                temp = Lerp(temp, pEndNextRoot, 0.5);
                bRoot = Lerp(pEndRoot, temp, -1.0 / 3);
            }
            interpolatedPosture.root_pos = DeCasteljauEuler(t, pStartRoot, aRoot, bRoot, pEndRoot);

            // interpolate bone rotations
            for (int bone = 0; bone < MAX_BONES_IN_ASF_FILE; bone++)
            {
                vector pStart, pEnd, a, b;
                pStart = startPosture->bone_rotation[bone];
                pEnd = endPosture->bone_rotation[bone];
                // detect edge case (start frame is 0, or end frame is N)
                if (startKeyframe == 0)
                {
                    vector pEndNext = pInputMotion->GetPosture(endKeyframe + N + 1)->bone_rotation[bone];
                    vector temp = Lerp(pEndNext, pEnd, 2.0);
                    a = Lerp(pStart, temp, 1.0 / 3);
                }
                else
                {
                    vector pStartPrevious = pInputMotion->GetPosture(startKeyframe - N - 1)->bone_rotation[bone];
                    vector temp = Lerp(pStartPrevious, pStart, 2.0);
                    temp = Lerp(temp, pEnd, 0.5);
                    a = Lerp(pStart, temp, 1.0 / 3);
                }

                if (endKeyframe + N + 1 > inputLength)
                {
                    vector pStartPrevious = pInputMotion->GetPosture(startKeyframe - N - 1)->bone_rotation[bone];
                    vector temp = Lerp(pStartPrevious, pStart, 2.0);
                    b = Lerp(pEnd, temp, 1.0 / 3);
                }
                else
                {
                    vector pEndNext = pInputMotion->GetPosture(endKeyframe + N + 1)->bone_rotation[bone];
                    vector temp = Lerp(pStart, pEnd, 2.0);
                    temp = Lerp(temp, pEndNext, 0.5);
                    b = Lerp(pEnd, temp, -1.0 / 3);
                }
                interpolatedPosture.bone_rotation[bone] = DeCasteljauEuler(t, pStart, a, b, pEnd);
            }

            pOutputMotion->SetPosture(startKeyframe + frame, interpolatedPosture);
        }

        startKeyframe = endKeyframe;
    }

    for (int frame = startKeyframe + 1; frame < inputLength; frame++)
        pOutputMotion->SetPosture(frame, *(pInputMotion->GetPosture(frame)));
}

void Interpolator::LinearInterpolationQuaternion(Motion * pInputMotion, Motion * pOutputMotion, int N)
{
    int inputLength = pInputMotion->GetNumFrames(); // frames are indexed 0, ..., inputLength-1

    int startKeyframe = 0;
    while (startKeyframe + N + 1 < inputLength)
    {
        int endKeyframe = startKeyframe + N + 1;

        Posture* startPosture = pInputMotion->GetPosture(startKeyframe);
        Posture* endPosture = pInputMotion->GetPosture(endKeyframe);

        // copy start and end keyframe
        pOutputMotion->SetPosture(startKeyframe, *startPosture);
        pOutputMotion->SetPosture(endKeyframe, *endPosture);

        // interpolate in between
        for (int frame = 1; frame <= N; frame++)
        {
            Posture interpolatedPosture;
            double t = 1.0 * frame / (N + 1);

            // interpolate root position
            interpolatedPosture.root_pos = startPosture->root_pos * (1 - t) + endPosture->root_pos * t;

            // interpolate bone rotations
            for (int bone = 0; bone < MAX_BONES_IN_ASF_FILE; bone++)
            {
                Quaternion<double> qStart, qEnd, qInterpolated;
                Euler2Quaternion(startPosture->bone_rotation[bone].p, qStart);
                Euler2Quaternion(endPosture->bone_rotation[bone].p, qEnd);
                // linear interpolate
                //qInterpolated = qStart * (1 - t) + qEnd * t;
                // slerp interpolate
                qInterpolated = Slerp(t, qStart, qEnd);
                double pInterpolated[3];
                Quaternion2Euler(qInterpolated, pInterpolated);
                interpolatedPosture.bone_rotation[bone] = pInterpolated;
            }

            pOutputMotion->SetPosture(startKeyframe + frame, interpolatedPosture);
        }

        startKeyframe = endKeyframe;
    }

    for (int frame = startKeyframe + 1; frame < inputLength; frame++)
        pOutputMotion->SetPosture(frame, *(pInputMotion->GetPosture(frame)));
}

void Interpolator::BezierInterpolationQuaternion(Motion * pInputMotion, Motion * pOutputMotion, int N)
{
    int inputLength = pInputMotion->GetNumFrames(); // frames are indexed 0, ..., inputLength-1

    int startKeyframe = 0;
    while (startKeyframe + N + 1 < inputLength)
    {
        int endKeyframe = startKeyframe + N + 1;

        Posture* startPosture = pInputMotion->GetPosture(startKeyframe);
        Posture* endPosture = pInputMotion->GetPosture(endKeyframe);

        // copy start and end keyframe
        pOutputMotion->SetPosture(startKeyframe, *startPosture);
        pOutputMotion->SetPosture(endKeyframe, *endPosture);

        // interpolate in between
        for (int frame = 1; frame <= N; frame++)
        {
            Posture interpolatedPosture;
            double t = 1.0 * frame / (N + 1);

            // interpolate root position
            //interpolatedPosture.root_pos = startPosture->root_pos * (1 - t) + endPosture->root_pos * t;
            vector pStartRoot, pEndRoot, aRoot, bRoot;
            pStartRoot = startPosture->root_pos;
            pEndRoot = endPosture->root_pos;
            // detect edge case (start frame is 0, or end frame is N)
            if (startKeyframe == 0)
            {
                vector pEndNextRoot = pInputMotion->GetPosture(endKeyframe + N + 1)->root_pos;
                vector temp = Lerp(pEndNextRoot, pEndRoot, 2.0);
                aRoot = Lerp(pStartRoot, temp, 1.0 / 3);
            }
            else
            {
                vector pStartPreviousRoot = pInputMotion->GetPosture(startKeyframe - N - 1)->root_pos;
                vector temp = Lerp(pStartPreviousRoot, pStartRoot, 2.0);
                temp = Lerp(temp, pEndRoot, 0.5);
                aRoot = Lerp(pStartRoot, temp, 1.0 / 3);
            }

            if (endKeyframe + N + 1 > inputLength)
            {
                vector pStartPreviousRoot = pInputMotion->GetPosture(startKeyframe - N - 1)->root_pos;
                vector temp = Lerp(pStartPreviousRoot, pStartRoot, 2.0);
                bRoot = Lerp(pEndRoot, temp, 1.0 / 3);
            }
            else
            {
                vector pEndNextRoot = pInputMotion->GetPosture(endKeyframe + N + 1)->root_pos;
                vector temp = Lerp(pStartRoot, pEndRoot, 2.0);
                temp = Lerp(temp, pEndNextRoot, 0.5);
                bRoot = Lerp(pEndRoot, temp, -1.0 / 3);
            }
            interpolatedPosture.root_pos = DeCasteljauEuler(t, pStartRoot, aRoot, bRoot, pEndRoot);

            // interpolate bone rotations
            for (int bone = 0; bone < MAX_BONES_IN_ASF_FILE; bone++)
            {
                Quaternion<double> qStart, qEnd, a, b;
                Euler2Quaternion(startPosture->bone_rotation[bone].p, qStart);
                Euler2Quaternion(endPosture->bone_rotation[bone].p, qEnd);
                // detect edge case (start frame is 0, or end frame is N)
                if (startKeyframe == 0)
                {
                    Quaternion<double> qEndNext;
                    Euler2Quaternion(pInputMotion->GetPosture(endKeyframe + N + 1)->bone_rotation[bone].p, qEndNext);
                    Quaternion<double> temp = Slerp(2.0, qEndNext, qEnd);
                    a = Slerp(1.0 / 3, qStart, temp);
                }
                else
                {
                    Quaternion<double> qStartPrevious;
                    Euler2Quaternion(pInputMotion->GetPosture(startKeyframe - N - 1)->bone_rotation[bone].p, qStartPrevious);
                    Quaternion<double> temp = Slerp(2.0, qStartPrevious, qStart);
                    temp = Slerp(0.5, temp, qEnd);
                    a = Slerp(1.0 / 3, qStart, temp);
                }

                if (endKeyframe + N + 1 > inputLength)
                {
                    Quaternion<double> qStartPrevious;
                    Euler2Quaternion(pInputMotion->GetPosture(startKeyframe - N - 1)->bone_rotation[bone].p, qStartPrevious);
                    Quaternion<double> temp = Slerp(2.0, qStartPrevious, qStart);
                    b = Slerp(1.0 / 3, qEnd, temp);
                }
                else
                {
                    Quaternion<double> qEndNext;
                    Euler2Quaternion(pInputMotion->GetPosture(endKeyframe + N + 1)->bone_rotation[bone].p, qEndNext);
                    Quaternion<double> temp = Slerp(2.0, qStart, qEnd);
                    temp = Slerp(0.5, temp, qEndNext);
                    b = Slerp(-1.0 / 3, qEnd, temp);
                }
                Quaternion<double> qInterpolated;
                qInterpolated = DeCasteljauQuaternion(t, qStart, a, b, qEnd);
                double pInterpolated[3];
                Quaternion2Euler(qInterpolated, pInterpolated);
                interpolatedPosture.bone_rotation[bone] = pInterpolated;
            }

            pOutputMotion->SetPosture(startKeyframe + frame, interpolatedPosture);
        }

        startKeyframe = endKeyframe;
    }

    for (int frame = startKeyframe + 1; frame < inputLength; frame++)
        pOutputMotion->SetPosture(frame, *(pInputMotion->GetPosture(frame)));
}

void Interpolator::Euler2Quaternion(double angles[3], Quaternion<double> & q) 
{
    double tempR[9];
    Euler2Rotation(angles, tempR);
    q = Quaternion<double>::Matrix2Quaternion(tempR);
    q.Normalize();
}

void Interpolator::Quaternion2Euler(Quaternion<double> & q, double angles[3]) 
{
    double tempR[9];
    q.Quaternion2Matrix(tempR);
    Rotation2Euler(tempR, angles);
}


// Reference: https://discourse.panda3d.org/t/shortest-rotation-arc-between-quaternions/26144/2
Quaternion<double> Interpolator::Slerp(double t, Quaternion<double> & qStart, Quaternion<double> & qEnd_)
{
  qStart.Normalize();
  qEnd_.Normalize();
  
  Quaternion<double> result;

  // cos(theta) = dot(q1, q2)
  double cosTheta = qStart.Getx() * qEnd_.Getx() + qStart.Gety() * qEnd_.Gety() + qStart.Getz() * qEnd_.Getz() + qStart.Gets() * qEnd_.Gets();

  // pick shortest path
  if (cosTheta < 0.0)
  {
      // negate one quaternion
      qEnd_.Set(-qEnd_.Gets(), -qEnd_.Getx(), -qEnd_.Gety(), -qEnd_.Getz());
      cosTheta = -cosTheta;
  }

  double theta = acos(cosTheta);

  // when theta is too small
  if (theta < 0.0001)
  {
      return qStart;
  }

  result = sin((1 - t) * theta) / sin(theta) * qStart + sin(t * theta) / sin(theta) * qEnd_;

  result.Normalize();
  
  return result;
}

Quaternion<double> Interpolator::Double(Quaternion<double> p, Quaternion<double> q)
{
  Quaternion<double> result;
  result = Slerp(2.0, p, q);
  return result;
}

vector Interpolator::DeCasteljauEuler(double t, vector p0, vector p1, vector p2, vector p3)
{
  vector result, q0, q1, q2, r0, r1;

  q0 = Lerp(p0, p1, t);
  q1 = Lerp(p1, p2, t);
  q2 = Lerp(p2, p3, t);
  r0 = Lerp(q0, q1, t);
  r1 = Lerp(q1, q2, t);
  result = Lerp(r0, r1, t);

  return result;
}

Quaternion<double> Interpolator::DeCasteljauQuaternion(double t, Quaternion<double> p0, Quaternion<double> p1, Quaternion<double> p2, Quaternion<double> p3)
{
  Quaternion<double> result, q0, q1, q2, r0, r1;

  q0 = Slerp(t, p0, p1);
  q1 = Slerp(t, p1, p2);
  q2 = Slerp(t, p2, p3);
  r0 = Slerp(t, q0, q1);
  r1 = Slerp(t, q1, q2);
  result = Slerp(t, r0, r1);

  return result;
}

