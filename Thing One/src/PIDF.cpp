// class PIDF {
//     private:
//         float kP = 0.0;
//         float kI = 0.0;
//         float kD = 0.0;
//         float KF = 0.0;
//     public:
//         void setConstants(float kp, float ki, float kd, float kf) {
//             this.kP = kp;
//             this.kI = ki;
//             this.kD = kd;
//             this.kF = kf;
//         }

//         void setKP(float val) {
//             kP = val;
//         }
//         float getKP () {
//             return kP;
//         }

//         void setKI(float val) {
//             kI = val;
//         }
//         float getKI () {
//             return kI;
//         }

//         void setKD(float val) {
//             kD = val;
//         }
//         float getKD () {
//             return kD;
//         }

//         void setKF(float val) {
//             kF = val;
//         }
//         float getKF () {
//             return kF;
//         }

//         float getPF(float pos, float targ) {

//             pow = 0.0;

//             // Apply P power
//             pow += kP * (pos - targ);

//             return pow;
//         }

// }