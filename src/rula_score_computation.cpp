/*
░█▀▄░█░█░█░░░█▀█░░░█▀▀░█▀▀░█▀█░█▀▄░█▀▀░░░█▀▀░█▀█░█▄█░█▀█░█░█░▀█▀░█▀█░▀█▀░▀█▀░█▀█░█▀█    
░█▀▄░█░█░█░░░█▀█░░░▀▀█░█░░░█░█░█▀▄░█▀▀░░░█░░░█░█░█░█░█▀▀░█░█░░█░░█▀█░░█░░░█░░█░█░█░█    
░▀░▀░▀▀▀░▀▀▀░▀░▀░░░▀▀▀░▀▀▀░▀░▀░▀░▀░▀▀▀░░░▀▀▀░▀▀▀░▀░▀░▀░░░▀▀▀░░▀░░▀░▀░░▀░░▀▀▀░▀▀▀░▀░▀    
*/

#include <algorithm>
#include <iostream>
#include <cmath>
#include <chrono>

#include "rula_score_computation.hpp"

// ─────────────────────────────────────────────────────────────────────────────
// Parameters
// ─────────────────────────────────────────────────────────────────────────────
int prevUpperArmScore = 0;
int prevNeckScore = 0;
int prevTrunkScore = 0;
int prevLegScore = 0;
auto startA = std::chrono::steady_clock::now();
auto startB = std::chrono::steady_clock::now();
bool started = false;


// ─────────────────────────────────────────────────────────────────────────────
// Conversions
// ─────────────────────────────────────────────────────────────────────────────
double angleDeg(const Vec3& a, const Vec3& b) {
    double c = a.normalized().dot(b.normalized());
    c = std::max(-1.0, std::min(1.0, c));   // clamp for numerical safety
    return std::acos(c) * 180.0 / M_PI;
}

Vec3 toVec3(const Eigen::Vector3d& p) {
    return {p[0], p[1], p[2]};
}

const Vec3 WORLD_UP = {0.0, 0.0, 1.0};


// ─────────────────────────────────────────────────────────────────────────────
// Group A scoring
// ─────────────────────────────────────────────────────────────────────────────
int scoreUpperArm(const Vec3& shoulder, const Vec3& elbow,
                   const Vec3& upperTorso, const Vec3& lowerTorso,
                   const AdjustmentFlags& f)
{
    // Vector from shoulder downward along trunk (reference for 0°)
    Vec3 trunkDown = (lowerTorso - upperTorso).normalized();
    Vec3 upperArm  = (elbow - shoulder).normalized();

    // Angle between upper arm and the trunk-down direction
    double ang = angleDeg(upperArm, trunkDown);

    // ang ≈ 0  → arm hanging straight down (neutral)
    // ang ≈ 90 → arm horizontal
    // ang ≈ 180→ arm fully raised overhead

    // RULA defines flexion relative to the trunk:
    // neutral (arm down) = 0°, fully raised forward = 180°
    // We map the raw angle to RULA convention:
    //   <20° ≈ arm near vertical → score 1
    int score;
    if      (ang <= 20)  score = 1;
    else if (ang <= 45)  score = 2;
    else if (ang <= 90)  score = 3;
    else                 score = 4;

    if (f.shoulderRaised)   ++score;
    if (f.upperArmAbducted) ++score;
    if (f.armSupported)     --score;

    return std::max(1, score);
}

int scoreLowerArm(const Vec3& shoulder, const Vec3& elbow,
                   const Vec3& wrist,
                   const AdjustmentFlags& f)
{
    Vec3 upper = (shoulder - elbow).normalized();
    Vec3 lower = (wrist    - elbow).normalized();
    double ang = angleDeg(upper, lower);

    // ang is the elbow flexion angle (0°=fully extended, 180°=fully flexed)
    // RULA uses flexion from 0°: score 1 for 60-100°, score 2 otherwise
    // Note: angleDeg gives supplementary angle relative to straight line.
    // elbow angle in RULA = 180° - ang (flexion from straight)
    double flexion = 180.0 - ang;

    int score = (flexion >= 60 && flexion <= 100) ? 1 : 2;

    if (f.crossingMidlineOrOut) ++score;

    return score;
}

int scoreWrist(const Vec3& elbow, const Vec3& wrist,
                const Vec3& hand,
                const AdjustmentFlags& f)
{
    // Approximate wrist flexion: angle between forearm and world vertical
    Vec3 forearm = (elbow - wrist).normalized();
    Vec3 handLine = (hand - wrist).normalized();
    double ang = angleDeg(forearm, handLine);

    // When forearm is horizontal (90° from up) and wrist is neutral,
    // deviation from 90° approximates flexion/extension
    double flexion = 180.0 - ang;

    int score;
    if      (flexion <= 5)  score = 1;   // near neutral
    else if (flexion <= 15) score = 2;
    else                          score = 3;

    if (f.wristDeviated) ++score;

    return score;
}

int scoreWristTwist(bool atEndOfRange) {
    return atEndOfRange ? 2 : 1;
}

static const int GROUP_A_TABLE[4][2][4][2] = {
    // Upper Arm 1
    { { {1,2},{2,2},{2,3},{3,3} },
      { {2,2},{2,2},{3,3},{3,3} } },
    // Upper Arm 2
    { { {2,3},{3,3},{3,3},{4,4} },
      { {3,3},{3,3},{3,4},{4,4} } },
    // Upper Arm 3
    { { {3,3},{4,4},{4,4},{5,5} },
      { {3,4},{4,4},{4,4},{5,5} } },
    // Upper Arm 4
    { { {4,4},{4,4},{4,5},{5,5} },
      { {4,4},{4,4},{5,5},{6,6} } }
};

int lookupGroupA(int upperArm, int lowerArm, int wrist, int wristTwist)
{
    int ua = std::min(std::max(upperArm,  1), 4) - 1;
    int la = std::min(std::max(lowerArm,  1), 2) - 1;
    int w  = std::min(std::max(wrist,     1), 4) - 1;
    int wt = std::min(std::max(wristTwist,1), 2) - 1;
    return GROUP_A_TABLE[ua][la][w][wt];
}


// ─────────────────────────────────────────────────────────────────────────────
// Group B scoring
// ─────────────────────────────────────────────────────────────────────────────
int scoreNeck(const Vec3& head, const Vec3& upperTorso,
               const Vec3& lowerTorso, const AdjustmentFlags& f)
{
    Vec3 neck = (head - upperTorso).normalized();
    Vec3 trunk = (upperTorso - lowerTorso).normalized();
    double ang = angleDeg(neck, trunk) - 10.0;

    // ang ≈ 0  → head straight up (neutral extension reference)
    // RULA flexion = angle forward from vertical
    int score;
    if      (ang <= 10) score = 1;
    else if (ang <= 20) score = 2;
    else if (ang <= 90) score = 3;
    else                score = 4;

    if (f.neckTwisted)   ++score;
    if (f.neckSideBent)  ++score;

    return score;
}

int scoreTrunk(const Vec3& upperTorso, const Vec3& lowerTorso,
                const AdjustmentFlags& f)
{
    Vec3 trunk = (upperTorso - lowerTorso).normalized();
    double ang = angleDeg(trunk, WORLD_UP) - 5.0;

    int score;
    if      (ang <= 5)  score = 1;   // well-supported / upright
    else if (ang <= 20) score = 2;
    else if (ang <= 60) score = 3;
    else                score = 4;

    if (f.trunkTwisted)   ++score;
    if (f.trunkSideBent)  ++score;

    return score;
}

int scoreLegs(const Vec3& lHip,  const Vec3& rHip,
               const Vec3& lKnee, const Vec3& rKnee,
               const Vec3& lAnkle,const Vec3& rAnkle)
{
    Vec3 lThigh  = (lKnee  - lHip).normalized();
    Vec3 lShank  = (lAnkle - lKnee).normalized();
    Vec3 rThigh  = (rKnee  - rHip).normalized();
    Vec3 rShank  = (rAnkle - rKnee).normalized();

    double lKneeAng = angleDeg(lThigh, lShank);
    double rKneeAng = angleDeg(rThigh, rShank);

    // If knees are roughly straight (legs extended / well-supported standing
    // or balanced sitting), score 1; otherwise score 2.
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
    // Neck 1
    { {1,3},{2,3},{3,4},{5,5},{7,7} },
    // Neck 2
    { {2,3},{2,3},{4,5},{5,6},{7,7} },
    // Neck 3
    { {3,3},{3,4},{5,6},{6,7},{7,8} },
    // Neck 4
    { {5,5},{5,6},{6,7},{7,8},{8,9} }
};

int lookupGroupB(int neck, int trunk, int legs)
{
    int n = std::min(std::max(neck,  1), 4) - 1;
    int t = std::min(std::max(trunk, 1), 5) - 1;
    int l = std::min(std::max(legs,  1), 2) - 1;
    return GROUP_B_TABLE[n][t][l];
}


// ─────────────────────────────────────────────────────────────────────────────
// Grand score
// ─────────────────────────────────────────────────────────────────────────────
static const int GRAND_SCORE_TABLE[8][8] = {
    {1, 2, 3, 3, 4, 5, 5},   // Score A = 1
    {2, 2, 3, 4, 4, 5, 5},   // Score A = 2
    {3, 3, 3, 4, 4, 5, 6},   // Score A = 3
    {3, 3, 3, 4, 5, 6, 6},   // Score A = 4
    {4, 4, 4, 5, 6, 7, 7},   // Score A = 5
    {4, 4, 5, 6, 6, 7, 7},   // Score A = 6
    {5, 5, 6, 6, 7, 7, 7},   // Score A = 7
    {5, 5, 6, 7, 7, 7, 7}    // Score A = 8+
};

int lookupGrandScore(int scoreA, int scoreB)
{
    int a = std::min(std::max(scoreA, 1), 8) - 1;
    int b = std::min(std::max(scoreB, 1), 7) - 1;
    return GRAND_SCORE_TABLE[a][b];
}


// ─────────────────────────────────────────────────────────────────────────────
// Action level interpretation
// ─────────────────────────────────────────────────────────────────────────────
std::string actionLevel(int grandScore)
{
    if      (grandScore <= 2) return "Level 1 – Acceptable posture; no action required.";
    else if (grandScore <= 4) return "Level 2 – Further investigation; changes may be needed.";
    else if (grandScore <= 6) return "Level 3 – Investigation and changes needed soon.";
    else                      return "Level 4 – Immediate investigation and changes required!";
}


// ─────────────────────────────────────────────────────────────────────────────
// Main structure print function
// ─────────────────────────────────────────────────────────────────────────────
void RULAResult::print() const {
    std::cout << "=== RULA Assessment ===\n\n";

    std::cout << "-- Group A (Upper Limb) --\n";
    std::cout << "  Upper Arm score : " << upperArmScore   << "\n";
    std::cout << "  Lower Arm score : " << lowerArmScore   << "\n";
    std::cout << "  Wrist score     : " << wristScore      << "\n";
    std::cout << "  Wrist Twist     : " << wristTwistScore << "\n";
    std::cout << "  Posture Score A : " << postureScoreA   << "\n";
    std::cout << "  Muscle Use A    : +" << muscleUseScoreA << "\n";
    std::cout << "  Force Score A   : +" << forceScoreA     << "\n";
    std::cout << "  >>> Final Score A: " << finalScoreA     << "\n\n";

    std::cout << "-- Group B (Neck/Trunk/Legs) --\n";
    std::cout << "  Neck score      : " << neckScore       << "\n";
    std::cout << "  Trunk score     : " << trunkScore      << "\n";
    std::cout << "  Leg score       : " << legScore        << "\n";
    std::cout << "  Posture Score B : " << postureScoreB   << "\n";
    std::cout << "  Muscle Use B    : +" << muscleUseScoreB << "\n";
    std::cout << "  Force Score B   : +" << forceScoreB     << "\n";
    std::cout << "  >>> Final Score B: " << finalScoreB     << "\n\n";

    std::cout << "=== Grand Score: " << grandScore << " ===\n";
    std::cout << action << "\n";
}




// ─────────────────────────────────────────────────────────────────────────────
// Static posture calculation for Group A
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
// Static posture calculation for Group B
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


// ─────────────────────────────────────────────────────────────────────────────
// Top-level function
// ─────────────────────────────────────────────────────────────────────────────
RULAResult computeRULA(const Skeleton& kp,
                        const AdjustmentFlags& f,
                        char side,
                        bool wristAtEndOfRange) {
    RULAResult r{};

    // Select left or right keypoints
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

    // --- Group A ---
    r.upperArmScore   = scoreUpperArm(shoulder, elbow, upperTorso, lowerTorso, f);
    r.lowerArmScore   = scoreLowerArm(shoulder, elbow, wrist, f);
    r.wristScore      = scoreWrist(elbow, wrist, hand, f);
    r.wristTwistScore = scoreWristTwist(wristAtEndOfRange);

    r.postureScoreA   = lookupGroupA(r.upperArmScore, r.lowerArmScore,
                                     r.wristScore,    r.wristTwistScore);   

    r.muscleUseScoreA = checkStaticGroupA(r.upperArmScore) + (f.isRepeated ? 1 : 0);
    r.forceScoreA     = f.forceScoreA;
    r.finalScoreA     = r.postureScoreA + r.muscleUseScoreA + r.forceScoreA;

    // --- Group B ---
    r.neckScore  = scoreNeck(head, upperTorso, lowerTorso, f);
    r.trunkScore = scoreTrunk(upperTorso, lowerTorso, f);
    r.legScore   = scoreLegs(toVec3(kp[L_HIP]), toVec3(kp[R_HIP]),
                              toVec3(kp[L_KNEE]), toVec3(kp[R_KNEE]),
                              toVec3(kp[L_ANKLE]), toVec3(kp[R_ANKLE]));

    r.postureScoreB   = lookupGroupB(r.neckScore, r.trunkScore, r.legScore);
    r.muscleUseScoreB = checkStaticGroupB(r.neckScore, r.trunkScore, r.legScore) + (f.isRepeated ? 1 : 0);
    r.forceScoreB     = f.forceScoreB;
    r.finalScoreB     = r.postureScoreB + r.muscleUseScoreB + r.forceScoreB;

    // --- Grand Score ---
    r.grandScore = lookupGrandScore(r.finalScoreA, r.finalScoreB);
    r.action     = actionLevel(r.grandScore);

    return r;
}