#pragma once
#include <Eigen/Core>
#include <cmath>
#include <vector>

// ─────────────────────────────────────────────────────────────────────────────
// Minimal 3D vector with the geometric operations used by the RULA scorer
// ─────────────────────────────────────────────────────────────────────────────
struct Vec3 {
    double x, y, z;

    Vec3 operator-(const Vec3& o) const { return {x-o.x, y-o.y, z-o.z}; }
    Vec3 operator+(const Vec3& o) const { return {x+o.x, y+o.y, z+o.z}; }
    Vec3 operator*(double s)      const { return {x*s,   y*s,   z*s};   }

    double dot(const Vec3& o)  const { return x*o.x + y*o.y + z*o.z; }
    double norm()              const { return std::sqrt(dot(*this));   }
    Vec3   normalized()        const { double n = norm(); return (n > 1e-9) ? (*this)*(1.0/n) : Vec3{0,0,0}; }
    Vec3   cross(const Vec3& o) const {
        return { y*o.z - z*o.y,
                 z*o.x - x*o.z,
                 x*o.y - y*o.x };
    }
};


// ── Conversions ─────────────────────────────────────────────────────────────
// ─────────────────────────────────────────────────────────────────────────────
// Angle in degrees between two vectors
// ─────────────────────────────────────────────────────────────────────────────
double angleDeg(const Vec3& a, const Vec3& b);

// ── Keypoint indices ────────────────────────────────────────────────────────
enum KP {
    HEAD=0,         L_SHOULDER=1,   R_SHOULDER=2,
    L_ELBOW=3,      R_ELBOW=4,
    L_WRIST=5,      R_WRIST=6,
    L_HAND=7,       R_HAND=8,
    UPPER_TORSO=9,  LOWER_TORSO=10,
    L_HIP=11,       R_HIP=12,
    L_KNEE=13,      R_KNEE=14,
    L_ANKLE=15,     R_ANKLE=16,
    L_HEEL=17,      R_HEEL=18,
    L_FOOT=19,      R_FOOT=20
};

using Skeleton = std::vector<Eigen::Vector3d>;

// ─────────────────────────────────────────────────────────────────────────────
// Convert an Eigen 3D point to a Vec3
// ─────────────────────────────────────────────────────────────────────────────
Vec3 toVec3(const Eigen::Vector3d& p);

extern const Vec3 WORLD_UP;


// ── Optional adjustment flags ───────────────────────────────────────────────
// ─────────────────────────────────────────────────────────────────────────────
// Posture details that cannot be inferred from the keypoints alone
// ─────────────────────────────────────────────────────────────────────────────
struct AdjustmentFlags {
    bool shoulderRaised    = false;
    bool upperArmAbducted  = false;
    bool armSupported      = false;

    bool crossingMidlineOrOut = false;

    bool wristDeviated     = false;

    bool neckTwisted       = false;
    bool neckSideBent      = false;

    bool trunkTwisted      = false;
    bool trunkSideBent     = false;

    bool isRepeated        = false;

    int  forceScoreA       = 0;
    int  forceScoreB       = 0;
};


// ── Group A scoring ─────────────────────────────────────────────────────────
// ─────────────────────────────────────────────────────────────────────────────
// Score the upper arm from the shoulder, elbow and torso landmarks
// ─────────────────────────────────────────────────────────────────────────────
int scoreUpperArm(const Vec3& shoulder, const Vec3& elbow,
                   const Vec3& upperTorso, const Vec3& lowerTorso,
                   const AdjustmentFlags& f);

// ─────────────────────────────────────────────────────────────────────────────
// Score the lower arm from the shoulder, elbow and wrist landmarks
// ─────────────────────────────────────────────────────────────────────────────
int scoreLowerArm(const Vec3& shoulder, const Vec3& elbow,
                   const Vec3& wrist, const AdjustmentFlags& f);

// ─────────────────────────────────────────────────────────────────────────────
// Score the wrist from the elbow, wrist and hand landmarks
// ─────────────────────────────────────────────────────────────────────────────
int scoreWrist(const Vec3& elbow, const Vec3& wrist,
                const Vec3& hand, const AdjustmentFlags& f);

// ─────────────────────────────────────────────────────────────────────────────
// Score the wrist twist from its end-of-range state
// ─────────────────────────────────────────────────────────────────────────────
int scoreWristTwist(bool atEndOfRange);

// ─────────────────────────────────────────────────────────────────────────────
// Look up the Group A posture score from its four sub-scores
// ─────────────────────────────────────────────────────────────────────────────
int lookupGroupA(int upperArm, int lowerArm, int wrist, int wristTwist);


// ── Group B scoring ─────────────────────────────────────────────────────────
// ─────────────────────────────────────────────────────────────────────────────
// Score the neck from the head and torso landmarks
// ─────────────────────────────────────────────────────────────────────────────
int scoreNeck(const Vec3& head, const Vec3& upperTorso,
               const Vec3& lowerTorso, const AdjustmentFlags& f);

// ─────────────────────────────────────────────────────────────────────────────
// Score the trunk from the torso landmarks
// ─────────────────────────────────────────────────────────────────────────────
int scoreTrunk(const Vec3& upperTorso, const Vec3& lowerTorso,
                const AdjustmentFlags& f);

// ─────────────────────────────────────────────────────────────────────────────
// Score the legs from the hip, knee and ankle landmarks
// ─────────────────────────────────────────────────────────────────────────────
int scoreLegs(const Vec3& lHip,  const Vec3& rHip,
               const Vec3& lKnee, const Vec3& rKnee,
               const Vec3& lAnkle,const Vec3& rAnkle);

// ─────────────────────────────────────────────────────────────────────────────
// Look up the Group B posture score from its three sub-scores
// ─────────────────────────────────────────────────────────────────────────────
int lookupGroupB(int neck, int trunk, int legs);


// ── Grand score ─────────────────────────────────────────────────────────────
// ─────────────────────────────────────────────────────────────────────────────
// Look up the grand RULA score from the Group A and Group B scores
// ─────────────────────────────────────────────────────────────────────────────
int lookupGrandScore(int scoreA, int scoreB);


// ── Main structure ──────────────────────────────────────────────────────────
// ─────────────────────────────────────────────────────────────────────────────
// All intermediate and final RULA scores for one side of the body
// ─────────────────────────────────────────────────────────────────────────────
struct RULAResult {
    int upperArmScore, lowerArmScore, wristScore, wristTwistScore;
    int postureScoreA, muscleUseScoreA, forceScoreA, finalScoreA;

    int neckScore, trunkScore, legScore;
    int postureScoreB, muscleUseScoreB, forceScoreB, finalScoreB;

    int grandScore;
};


// ── Top-level function ──────────────────────────────────────────────────────
// ─────────────────────────────────────────────────────────────────────────────
// Compute the full RULA result for a skeleton, side and adjustment flags
// ─────────────────────────────────────────────────────────────────────────────
RULAResult computeRULA(const Skeleton& kp,
                        const AdjustmentFlags& f,
                        char side = 'R',
                        bool wristAtEndOfRange = false);
