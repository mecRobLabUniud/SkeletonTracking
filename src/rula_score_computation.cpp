/*
░█▀▄░█░█░█░░░█▀█░░░█▀▀░█▀▀░█▀█░█▀▄░█▀▀░░░█▀▀░█▀█░█▄█░█▀█░█░█░▀█▀░█▀█░▀█▀░▀█▀░█▀█░█▀█    
░█▀▄░█░█░█░░░█▀█░░░▀▀█░█░░░█░█░█▀▄░█▀▀░░░█░░░█░█░█░█░█▀▀░█░█░░█░░█▀█░░█░░░█░░█░█░█░█    
░▀░▀░▀▀▀░▀▀▀░▀░▀░░░▀▀▀░▀▀▀░▀░▀░▀░▀░▀▀▀░░░▀▀▀░▀▀▀░▀░▀░▀░░░▀▀▀░░▀░░▀░▀░░▀░░▀▀▀░▀▀▀░▀░▀    
*/

#include <algorithm>
#include <cmath>
#include <chrono>

#include "rula_score_computation.hpp"

// ── Parameters and shared state ─────────────────────────────────────────────
int prevUpperArmScore = 0;
int prevNeckScore = 0;
int prevTrunkScore = 0;
int prevLegScore = 0;
auto startA = std::chrono::steady_clock::now();
auto startB = std::chrono::steady_clock::now();
bool started = false;


// ── Conversions ─────────────────────────────────────────────────────────────
// ─────────────────────────────────────────────────────────────────────────────
// Angle in degrees between two vectors
// ─────────────────────────────────────────────────────────────────────────────
double angleDeg(const Vec3& a, const Vec3& b) {
    double c = a.normalized().dot(b.normalized());
    c = std::max(-1.0, std::min(1.0, c));
    return std::acos(c) * 180.0 / M_PI;
}

// ─────────────────────────────────────────────────────────────────────────────
// Convert an Eigen 3D point to a Vec3
// ─────────────────────────────────────────────────────────────────────────────
Vec3 toVec3(const Eigen::Vector3d& p) {
    return {p[0], p[1], p[2]};
}

const Vec3 WORLD_UP = {0.0, 0.0, 1.0};


// ── Group A scoring ─────────────────────────────────────────────────────────
// ─────────────────────────────────────────────────────────────────────────────
// Score the upper arm from the shoulder, elbow and torso landmarks
// ─────────────────────────────────────────────────────────────────────────────
int scoreUpperArm(const Vec3& shoulder, const Vec3& elbow,
                   const Vec3& upperTorso, const Vec3& lowerTorso,
                   const AdjustmentFlags& f)
{
    // ── Flexion angle relative to the trunk ─────────────────────────────────
    Vec3 trunkDown = (lowerTorso - upperTorso).normalized();
    Vec3 upperArm  = (elbow - shoulder).normalized();

    double ang = angleDeg(upperArm, trunkDown);

    int score;
    if      (ang <= 20)  score = 1;
    else if (ang <= 45)  score = 2;
    else if (ang <= 90)  score = 3;
    else                 score = 4;

    // ── Adjustment flags ────────────────────────────────────────────────────
    if (f.shoulderRaised)   ++score;
    if (f.upperArmAbducted) ++score;
    if (f.armSupported)     --score;

    return std::max(1, score);
}

// ─────────────────────────────────────────────────────────────────────────────
// Score the lower arm from the shoulder, elbow and wrist landmarks
// ─────────────────────────────────────────────────────────────────────────────
int scoreLowerArm(const Vec3& shoulder, const Vec3& elbow,
                   const Vec3& wrist,
                   const AdjustmentFlags& f)
{
    // ── Elbow flexion angle ─────────────────────────────────────────────────
    Vec3 upper = (shoulder - elbow).normalized();
    Vec3 lower = (wrist    - elbow).normalized();
    double ang = angleDeg(upper, lower);

    double flexion = 180.0 - ang;

    int score = (flexion >= 60 && flexion <= 100) ? 1 : 2;

    // ── Adjustment flags ────────────────────────────────────────────────────
    if (f.crossingMidlineOrOut) ++score;

    return score;
}

// ─────────────────────────────────────────────────────────────────────────────
// Score the wrist from the elbow, wrist and hand landmarks
// ─────────────────────────────────────────────────────────────────────────────
int scoreWrist(const Vec3& elbow, const Vec3& wrist,
                const Vec3& hand,
                const AdjustmentFlags& f)
{
    // ── Wrist flexion angle ─────────────────────────────────────────────────
    Vec3 forearm = (elbow - wrist).normalized();
    Vec3 handLine = (hand - wrist).normalized();
    double ang = angleDeg(forearm, handLine);

    double flexion = 180.0 - ang;

    int score;
    if      (flexion <= 5)  score = 1;
    else if (flexion <= 15) score = 2;
    else                    score = 3;

    // ── Adjustment flags ────────────────────────────────────────────────────
    if (f.wristDeviated) ++score;

    return score;
}

// ─────────────────────────────────────────────────────────────────────────────
// Score the wrist twist from its end-of-range state
// ─────────────────────────────────────────────────────────────────────────────
int scoreWristTwist(bool atEndOfRange) {
    return atEndOfRange ? 2 : 1;
}

static const int GROUP_A_TABLE[4][2][4][2] = {
    { { {1,2},{2,2},{2,3},{3,3} },
      { {2,2},{2,2},{3,3},{3,3} } },
    { { {2,3},{3,3},{3,3},{4,4} },
      { {3,3},{3,3},{3,4},{4,4} } },
    { { {3,3},{4,4},{4,4},{5,5} },
      { {3,4},{4,4},{4,4},{5,5} } },
    { { {4,4},{4,4},{4,5},{5,5} },
      { {4,4},{4,4},{5,5},{6,6} } }
};

// ─────────────────────────────────────────────────────────────────────────────
// Look up the Group A posture score from its four sub-scores
// ─────────────────────────────────────────────────────────────────────────────
int lookupGroupA(int upperArm, int lowerArm, int wrist, int wristTwist)
{
    int ua = std::min(std::max(upperArm,  1), 4) - 1;
    int la = std::min(std::max(lowerArm,  1), 2) - 1;
    int w  = std::min(std::max(wrist,     1), 4) - 1;
    int wt = std::min(std::max(wristTwist,1), 2) - 1;
    return GROUP_A_TABLE[ua][la][w][wt];
}


// ── Group B scoring ─────────────────────────────────────────────────────────
// ─────────────────────────────────────────────────────────────────────────────
// Score the neck from the head and torso landmarks
// ─────────────────────────────────────────────────────────────────────────────
int scoreNeck(const Vec3& head, const Vec3& upperTorso,
               const Vec3& lowerTorso, const AdjustmentFlags& f)
{
    // ── Neck flexion angle ──────────────────────────────────────────────────
    Vec3 neck = (head - upperTorso).normalized();
    Vec3 trunk = (upperTorso - lowerTorso).normalized();
    double ang = angleDeg(neck, trunk) - 10.0;

    int score;
    if      (ang <= 10) score = 1;
    else if (ang <= 20) score = 2;
    else if (ang <= 90) score = 3;
    else                score = 4;

    // ── Adjustment flags ────────────────────────────────────────────────────
    if (f.neckTwisted)   ++score;
    if (f.neckSideBent)  ++score;

    return score;
}

// ─────────────────────────────────────────────────────────────────────────────
// Score the trunk from the torso landmarks
// ─────────────────────────────────────────────────────────────────────────────
int scoreTrunk(const Vec3& upperTorso, const Vec3& lowerTorso,
                const AdjustmentFlags& f)
{
    // ── Trunk flexion angle ─────────────────────────────────────────────────
    Vec3 trunk = (upperTorso - lowerTorso).normalized();
    double ang = angleDeg(trunk, WORLD_UP) - 5.0;

    int score;
    if      (ang <= 5)  score = 1;
    else if (ang <= 20) score = 2;
    else if (ang <= 60) score = 3;
    else                score = 4;

    // ── Adjustment flags ────────────────────────────────────────────────────
    if (f.trunkTwisted)   ++score;
    if (f.trunkSideBent)  ++score;

    return score;
}

// ─────────────────────────────────────────────────────────────────────────────
// Score the legs from the hip, knee and ankle landmarks
// ─────────────────────────────────────────────────────────────────────────────
int scoreLegs(const Vec3& lHip,  const Vec3& rHip,
               const Vec3& lKnee, const Vec3& rKnee,
               const Vec3& lAnkle,const Vec3& rAnkle)
{
    // ── Knee angles ─────────────────────────────────────────────────────────
    Vec3 lThigh  = (lKnee  - lHip).normalized();
    Vec3 lShank  = (lAnkle - lKnee).normalized();
    Vec3 rThigh  = (rKnee  - rHip).normalized();
    Vec3 rShank  = (rAnkle - rKnee).normalized();

    double lKneeAng = angleDeg(lThigh, lShank);
    double rKneeAng = angleDeg(rThigh, rShank);

    // ── Standing / sitting balance score ────────────────────────────────────
    int score = 1;
    if (!std::isnan(lKnee.x) && !std::isnan(lAnkle.x)) {
        bool balanced = (lKneeAng < 30 || lKneeAng > 150);
        score = balanced ? 1 : 2;
    }
    if (!std::isnan(rKnee.x) && !std::isnan(rAnkle.x)) {
        bool balanced = (rKneeAng < 30 || rKneeAng > 150);
        score = std::max(score, balanced ? 1 : 2);
    }
    return score;
}

static const int GROUP_B_TABLE[4][5][2] = {
    { {1,3},{2,3},{3,4},{5,5},{7,7} },
    { {2,3},{2,3},{4,5},{5,6},{7,7} },
    { {3,3},{3,4},{5,6},{6,7},{7,8} },
    { {5,5},{5,6},{6,7},{7,8},{8,9} }
};

// ─────────────────────────────────────────────────────────────────────────────
// Look up the Group B posture score from its three sub-scores
// ─────────────────────────────────────────────────────────────────────────────
int lookupGroupB(int neck, int trunk, int legs)
{
    int n = std::min(std::max(neck,  1), 4) - 1;
    int t = std::min(std::max(trunk, 1), 5) - 1;
    int l = std::min(std::max(legs,  1), 2) - 1;
    return GROUP_B_TABLE[n][t][l];
}


// ── Grand score ─────────────────────────────────────────────────────────────
static const int GRAND_SCORE_TABLE[8][8] = {
    {1, 2, 3, 3, 4, 5, 5},
    {2, 2, 3, 4, 4, 5, 5},
    {3, 3, 3, 4, 4, 5, 6},
    {3, 3, 3, 4, 5, 6, 6},
    {4, 4, 4, 5, 6, 7, 7},
    {4, 4, 5, 6, 6, 7, 7},
    {5, 5, 6, 6, 7, 7, 7},
    {5, 5, 6, 7, 7, 7, 7}
};

// ─────────────────────────────────────────────────────────────────────────────
// Look up the grand RULA score from the Group A and Group B scores
// ─────────────────────────────────────────────────────────────────────────────
int lookupGrandScore(int scoreA, int scoreB)
{
    int a = std::min(std::max(scoreA, 1), 8) - 1;
    int b = std::min(std::max(scoreB, 1), 7) - 1;
    return GRAND_SCORE_TABLE[a][b];
}


// ─────────────────────────────────────────────────────────────────────────────
// Accumulated static-posture score for Group A
// ─────────────────────────────────────────────────────────────────────────────
int checkStaticGroupA(int upperArmScore) {
    if (upperArmScore > 2) {
        if (prevUpperArmScore <= 2) {
            startA = std::chrono::steady_clock::now();
        }
        auto end = std::chrono::steady_clock::now();
        double elapsed = std::chrono::duration<double>(end - startA).count();

        if (elapsed > 5.0) return 1;
    }
    prevUpperArmScore = upperArmScore;
    return 0;
}


// ─────────────────────────────────────────────────────────────────────────────
// Accumulated static-posture score for Group B
// ─────────────────────────────────────────────────────────────────────────────
int checkStaticGroupB(int neckScore, int trunkScore, int legScore) {
    if (neckScore > 2) {
        if (prevNeckScore <= 2 && !started) {
            startB = std::chrono::steady_clock::now();
            started = true;
        }
        auto end = std::chrono::steady_clock::now();
        double elapsed = std::chrono::duration<double>(end - startB).count();

        if (elapsed > 5.0) return 1; else return 0;
    }
    else if (trunkScore > 2) {
        if (prevTrunkScore <= 2 && !started) {
            startB = std::chrono::steady_clock::now();
            started = true;
        }
        auto end = std::chrono::steady_clock::now();
        double elapsed = std::chrono::duration<double>(end - startB).count();

        if (elapsed > 5.0) return 1; else return 0;
    }
    else if (legScore > 1) {
        if (prevLegScore == 1 && !started) {
            startB = std::chrono::steady_clock::now();
            started = true;
        }
        auto end = std::chrono::steady_clock::now();
        double elapsed = std::chrono::duration<double>(end - startB).count();

        if (elapsed > 5.0) return 1; else return 0;
    }
    prevNeckScore = neckScore;
    prevTrunkScore = trunkScore;
    prevLegScore = legScore;
    started = false;
    return 0;
}


// ── Top-level function ──────────────────────────────────────────────────────
// ─────────────────────────────────────────────────────────────────────────────
// Compute the full RULA result for a skeleton, side and adjustment flags
// ─────────────────────────────────────────────────────────────────────────────
RULAResult computeRULA(const Skeleton& kp,
                        const AdjustmentFlags& f,
                        char side,
                        bool wristAtEndOfRange) {
    RULAResult r{};

    // ── Left/right keypoint selection ───────────────────────────────────────
    int shoulder_idx    = (side == 'L') ? L_SHOULDER    : R_SHOULDER;
    int elbow_idx       = (side == 'L') ? L_ELBOW       : R_ELBOW;
    int wrist_idx       = (side == 'L') ? L_WRIST       : R_WRIST;
    int hand_idx        = (side == 'L') ? L_HAND        : R_HAND;

    const Vec3 shoulder    = toVec3(kp[shoulder_idx]);
    const Vec3 elbow       = toVec3(kp[elbow_idx]);
    const Vec3 wrist       = toVec3(kp[wrist_idx]);
    const Vec3 hand       = toVec3(kp[hand_idx]);
    const Vec3 upperTorso  = toVec3(kp[UPPER_TORSO]);
    const Vec3 lowerTorso  = toVec3(kp[LOWER_TORSO]);
    const Vec3 head        = toVec3(kp[HEAD]);

    // ── Group A ─────────────────────────────────────────────────────────────
    r.upperArmScore   = scoreUpperArm(shoulder, elbow, upperTorso, lowerTorso, f);
    r.lowerArmScore   = scoreLowerArm(shoulder, elbow, wrist, f);
    r.wristScore      = scoreWrist(elbow, wrist, hand, f);
    r.wristTwistScore = scoreWristTwist(wristAtEndOfRange);

    r.postureScoreA   = lookupGroupA(r.upperArmScore, r.lowerArmScore,
                                     r.wristScore,    r.wristTwistScore);   

    r.muscleUseScoreA = checkStaticGroupA(r.upperArmScore) + (f.isRepeated ? 1 : 0);
    r.forceScoreA     = f.forceScoreA;
    r.finalScoreA     = r.postureScoreA + r.muscleUseScoreA + r.forceScoreA;

    // ── Group B ─────────────────────────────────────────────────────────────
    r.neckScore  = scoreNeck(head, upperTorso, lowerTorso, f);
    r.trunkScore = scoreTrunk(upperTorso, lowerTorso, f);
    r.legScore   = scoreLegs(toVec3(kp[L_HIP]), toVec3(kp[R_HIP]),
                              toVec3(kp[L_KNEE]), toVec3(kp[R_KNEE]),
                              toVec3(kp[L_ANKLE]), toVec3(kp[R_ANKLE]));

    r.postureScoreB   = lookupGroupB(r.neckScore, r.trunkScore, r.legScore);
    r.muscleUseScoreB = checkStaticGroupB(r.neckScore, r.trunkScore, r.legScore) + (f.isRepeated ? 1 : 0);
    r.forceScoreB     = f.forceScoreB;
    r.finalScoreB     = r.postureScoreB + r.muscleUseScoreB + r.forceScoreB;

    // ── Grand score ─────────────────────────────────────────────────────────
    r.grandScore = lookupGrandScore(r.finalScoreA, r.finalScoreB);

    return r;
}
