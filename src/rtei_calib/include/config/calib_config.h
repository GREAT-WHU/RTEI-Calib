//
// Created by csl on 10/1/22.
// Revised by lsy on 14/2/24.
//

#ifndef RTEI_CALIB_CALIB_CONFIG_H
#define RTEI_CALIB_CALIB_CONFIG_H

#include "fmt/format.h"
#include "sophus/se3.hpp"
#include "string"
#include "thirdparty/logger/src/include/logger.h"
#include "util/macros.hpp"
#include "util/type_define.hpp"
#include "opencv2/highgui.hpp"

namespace ns_eicalib {

    using namespace magic_enum::bitwise_operators;

    struct SolveModeConfig {

        enum class Type : std::uint32_t {
            /**
             * @brief options
             */
            IMU = 1 << 1,
            Event = 1 <<2,
            MULTI_Event_IMU = IMU | Event,
        };

        static bool IsSolveModeTypeWith(Type desired, Type curSolveModeType) {
            return (desired == (desired & curSolveModeType));
        }

        /**
         * @brief override operator '<<' for type 'SolveModeType'
         */
        friend std::ostream &operator<<(std::ostream &os, const Type &curSolveModeType) {
            std::stringstream stream;
            if (IsSolveModeTypeWith(Type::MULTI_Event_IMU, curSolveModeType)){
                stream << "MULTI_Event_IMU";
            }
            else {
                stream << "unknown solve mode";
            }
            os << stream.str();
            return os;
        }
    };

    struct CamIntrinsPar {
    public:
        static int CamHeight;
        static int CamWidth;
        static double Fx;
        static double Fy;
        static double Cx;
        static double Cy;
        static double K1;
        static double K2;
        static double P1;
        static double P2;
        static double K3;
        static double P_Fx;
        static double P_Fy;
        static double P_Cx;
        static double P_Cy;
        static cv::Matx33d CamMatrix;
    };

    struct CalibConfig {
    public:

    public:

        struct Optimization {
            static bool UseCuda;
            static bool LockTimeOffset;
            static bool LockIMUIntrinsic;
            struct OptWeight {
                static double GyroWeight;
                static double AcceWeight;

                static double SO3Weight;
                static double POSWeight;

                static double AngVelWeight;
            };
            static int ThreadNum;
            static int CeresIterations;
            static double TimeOffsetPadding;
            static double TimeReadoutPadding;
            static bool ProgressToStdout;
            static int RefineIterations;
        };

        struct EventCamInfo {
            static int NumEventPerPacket;
            static double OutputAngVelFrequency;
            static int EventBatchSize;
            static int BlurSigma;            
        };
        

        struct BSpline {
            static double KnotTimeDistance;
            // the order(degree + 1) of the b-spline
            // for simulate
            // static constexpr int SplineOrder = 10;
            // for solving
            static constexpr int SplineOrder = 4;
            static constexpr double GRefineNorm = 9.797;
        };

        struct CalibData {
            struct Topic {
                static std::set<std::string> IMUTopics;
                static std::vector<std::string> EventTopics;
            };
            static std::string BagPath;
            static std::string ParamSavePath;

            static double BeginTime;
            static double Duration;

            static std::map<std::string, CamIntrinsPar> CamParVecs;

            struct OutputData {
                static std::string OutputDataDir;
                static bool OutputIMUFrame;
                static bool OutputWarpedImageFrame;

                static bool OutputLMEquationGraph;
                static bool OutputParamInEachIter;

                static std::string OutputIMUFrameDir;

                // [ topic, directory ]
                static std::map<std::string, std::string> OutputIMUFrameDirs;

                // the precision for float data output
                static constexpr int Precision = 12;
            };

        };

        static SolveModeConfig::Type SolveMode;

    private:
        static bool _loadFinished;
        static bool _checkFinished;

    public:
        // load configure information from the yaml file
        static bool LoadConfigure(const std::string &filename);

        // check the config loading status, make sure call it  every time before using CalibConfig info
        static void CheckConfigureStatus();

        static bool IMUIntegrated();

        static bool EventIntegrated();

    protected:
        // check the parameters, if there is any invalid parameter, it will throw an exception
        static void CheckConfigure();

        // print the main fields for configure check
        static void PrintMainFields();

    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    };
}

#endif //RTEI_CALIB_CALIB_CONFIG_H