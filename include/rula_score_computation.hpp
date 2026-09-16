#pragma once
#include <Eigen/Core>
#include <cmath>
#include <string>
#include <vector>

// ─────────────────────────────────────────────────────────────────────────────
// Types
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


// ─────────────────────────────────────────────────────────────────────────────
// Conversions
// ─────────────────────────────────────────────────────────────────────────────
double angleDeg(const Vec3& a, const Vec3& b);

// Keypoint indices
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

// Convert a raw keypoint array to Vec3
Vec3 toVec3(const Eigen::Vector3d& p);

extern const Vec3 WORLD_UP;


// ─────────────────────────────────────────────────────────────────────────────
// Optional adjustement flags
// ─────────────────────────────────────────────────────────────────────────────
struct AdjustmentFlags {
    // Group A – Upper Arm
    bool shoulderRaised    = false;   // +1
    bool upperArmAbducted  = false;   // +1
    bool armSupported      = false;   // -1

    // Group A – Lower Arm
    bool crossingMidlineOrOut = false; // +1

    // Group A – Wrist
    bool wristDeviated     = false;   // +1 (radial/ulnar)

    // Group B – Neck
    bool neckTwisted       = false;   // +1
    bool neckSideBent      = false;   // +1

    // Group B – Trunk
    bool trunkTwisted      = false;   // +1
    bool trunkSideBent     = false;   // +1

    // Muscle use & force (applied identically to both groups A and B)
    bool isStaticPosture   = false;   // held >1 min  → +1
    bool isRepeated        = false;   // >4 times/min → +1

    // Force/load score (0–3) for group A and B independently
    int  forceScoreA       = 0;       // 0=<2kg intermittent … 3=shock/rapid
    int  forceScoreB       = 0;
};


// ─────────────────────────────────────────────────────────────────────────────
// Group A scoring
// ─────────────────────────────────────────────────────────────────────────────
int scoreUpperArm(const Vec3& shoulder, const Vec3& elbow,
                   const Vec3& upperTorso, const Vec3& lowerTorso,
                   const AdjustmentFlags& f);

int scoreLowerArm(const Vec3& shoulder, const Vec3& elbow,
                   const Vec3& wrist, const AdjustmentFlags& f);

int scoreWrist(const Vec3& elbow, const Vec3& wrist,
                const Vec3& hand, const AdjustmentFlags& f);

int scoreWristTwist(bool atEndOfRange);

int lookupGroupA(int upperArm, int lowerArm, int wrist, int wristTwist);


// ─────────────────────────────────────────────────────────────────────────────
// Group B scoring
// ─────────────────────────────────────────────────────────────────────────────
int scoreNeck(const Vec3& head, const Vec3& upperTorso,
               const Vec3& lowerTorso, const AdjustmentFlags& f);

int scoreTrunk(const Vec3& upperTorso, const Vec3& lowerTorso,
                const AdjustmentFlags& f);

int scoreLegs(const Vec3& lHip,  const Vec3& rHip,
               const Vec3& lKnee, const Vec3& rKnee,
               const Vec3& lAnkle,const Vec3& rAnkle);

int lookupGroupB(int neck, int trunk, int legs);


// ─────────────────────────────────────────────────────────────────────────────
// Grand score
// ─────────────────────────────────────────────────────────────────────────────
int lookupGrandScore(int scoreA, int scoreB);


// ─────────────────────────────────────────────────────────────────────────────
// Action level interpretation
// ─────────────────────────────────────────────────────────────────────────────
std::string actionLevel(int grandScore);


// ─────────────────────────────────────────────────────────────────────────────
// Main structure
// ─────────────────────────────────────────────────────────────────────────────
struct RULAResult {
    // Group A intermediates
    int upperArmScore, lowerArmScore, wristScore, wristTwistScore;
    int postureScoreA, muscleUseScoreA, forceScoreA, finalScoreA;

    // Group B intermediates
    int neckScore, trunkScore, legScore;
    int postureScoreB, muscleUseScoreB, forceScoreB, finalScoreB;

    // Grand score
    int grandScore;
    std::string action;

    void print() const;
};


// ─────────────────────────────────────────────────────────────────────────────
// Top-level function
// ─────────────────────────────────────────────────────────────────────────────
/**
 * @param kp       15-element array of 3D keypoints (see enum KP for indices)
 * @param f        Adjustment flags (posture details not inferrable from kp)
 * @param side     Which side to assess: 'L' or 'R' (RULA is per-side)
 * @param wristAtEndOfRange  true if wrist is at end of rotation range
 */
RULAResult computeRULA(const Skeleton& kp,
                        const AdjustmentFlags& f,
                        char side = 'R',
                        bool wristAtEndOfRange = false);