#pragma once

#include <ros/ros.h>
#include <fstream>
#include <string>
#include <iostream>
#include <vector>
#include <ceres/ceres.h>
#include <Eigen/Core>
#include <chrono>
#include <pangolin/pangolin.h>

namespace CUADC{

struct AcroDynamicParamters
{
    double alpha, Beta, CL, CDi, CDv, CD, CY, Cl, Cm, Cn, Cni, QInf, XCP;
};

struct FixWingParam
{
    // 副翼 升降舵 方向 空速 迎角
    AcroDynamicParamters paramters[3][3][3][5][91];  
};

struct Error
{
    std::string error;
    int count;
};

struct CURVE_FITTING_COST_CX_
{
    CURVE_FITTING_COST_CX_(double A, double E, double R, double V, double alpha, double CX):_A(A), _E(E),
                        _R(R), _1_V(1 / V), _alpha(alpha), _CX(CX)
                       {}
    
    // 残差计算
    template<typename T>
    bool operator()(
        const T *const CX_p, T* residual
    ) const{
        // CX - fX
        /*
             CX_p = CX_alpha0, CX_alpha, CX_alpha2, CXs_alpha3,
                    CX_A0, CX_A, CX_A, CX_A2, CX_A3, 
                    CX_E0, CX_E, CX_E2, CX_E3, 
                    CX_R0, CX_R, CX_R2, CX_R3, 
                    CX_1_V0, CX_1_V, CX_1_V2, CX_1_V3)^T
        */
        residual[0] = T(_CX) - 
                      (CX_p[0] + CX_p[1] * _alpha + CX_p[2] * pow(_alpha, 2) + CX_p[3] * pow(_alpha, 3)) *
                      (CX_p[4] + CX_p[5] * _A + CX_p[6] * pow(_A, 2) + CX_p[7] * pow(_A, 3)) *
                      (CX_p[8] + CX_p[9] * _E + CX_p[10] * pow(_E, 2) + CX_p[11] * pow(_E, 3)) *
                      (CX_p[12] + CX_p[13] * _R + CX_p[14] * pow(_R, 2) + CX_p[15] * pow(_R, 3)) *
                      (CX_p[16] + CX_p[17] * _1_V + CX_p[18] * pow(_1_V, 2) + CX_p[19] * pow(_1_V, 3));
        return true;

    }
    const double _A, _E, _R, _1_V, _alpha, _CX;
};

struct CURVE_FITTING_COST_CX
{
    CURVE_FITTING_COST_CX(double A, double E, double R, double V, double alpha, double CX):_A(A), _E(E),
                        _R(R), _1_V(1 / V), _alpha(alpha), _CX(CX)
                       {}
    
    // 残差计算
    template<typename T>
    bool operator()(
        const T *const CX_p, T* residual
    ) const{
        // CX - fX
        /*
            CX_p = (CX_0, 
                    CX_alpha, CX_alpha2 / 2, CXs_alpha3 / 6,
                    CX_A, CX_A2 / 2, CX_A3 / 6, 
                    CX_E, CX_E2 / 2, CX_E3 / 6, 
                    CX_R, CX_R2 / 2, CX_R3 / 6, 
                    CX_1_V, CX_1_V2 / 2, CX_1_V3 / 6)^T
        */
        residual[0] = T(_CX) - CX_p[0] - 
                      CX_p[1] * _alpha - CX_p[2] * pow(_alpha, 2) - CX_p[3] * pow(_alpha, 3) -
                      CX_p[4] * _A - CX_p[5] * pow (_A, 2) - CX_p[6] * pow (_A, 3) - 
                      CX_p[7] * _E - CX_p[8] * pow (_E, 2) - CX_p[9] * pow (_E, 3) -
                      CX_p[10] * _R - CX_p[11] * pow (_R, 2) - CX_p[12] * pow (_R, 3) -
                      CX_p[13] * _1_V - CX_p[14] * pow (_1_V, 2) - CX_p[15] * pow (_1_V, 3);
        return true;
    }
    const double _A, _E, _R, _1_V, _alpha, _CX;
};

struct CURVE_FITTING_COST_CX_2MIX
{
    CURVE_FITTING_COST_CX_2MIX(double A, double E, double R, double V, double alpha, double CX):_A(A), _E(E),
                        _R(R), _1_V(1 / V), _alpha(alpha), _CX(CX)
                       {}
    
    // 残差计算
    template<typename T>
    bool operator()(
        const T *const CX_p, T* residual
    ) const{
        // CX - fX
        /*
            CX_p_mix2 = (CX_0, 
                    CX_alpha, CX_alpha2, 
                    CX_A, CX_A2, 
                    CX_E, CX_E2, 
                    CX_R, CX_R2, 
                    CX_1_V, CX_1_V2, 
                    CX_AE, CX_AR, CX_A_1_V, CX_Aalpha, 
                    CX_ER, CX_E_1_V, CX_Ealpha, 
                    CX_R_1_V, CX_Ralpha, 
                    CX_1_Valpha)^T
        */
        residual[0] = T(_CX) - CX_p[0] - 
                      CX_p[1] * _alpha - CX_p[2] * pow(_alpha, 2) -
                      CX_p[3] * _A - CX_p[4] * pow (_A, 2) - 
                      CX_p[5] * _E - CX_p[6] * pow (_E, 2) -
                      CX_p[7] * _R - CX_p[8] * pow (_R, 2) -
                      CX_p[9] * _1_V - CX_p[10] * pow (_1_V, 2) -
                      CX_p[11] * _A * _E - CX_p[12] * _A * _R - CX_p[13] * _A * _1_V - CX_p[14] * _A * _alpha -
                      CX_p[15] * _E * _R - CX_p[16] * _E * _1_V - CX_p[17] * _E * _alpha -
                      CX_p[18] * _R * _1_V - CX_p[19] * _R * _alpha -
                      CX_p[20] * _1_V * _alpha;
        return true;
    }
    const double _A, _E, _R, _1_V, _alpha, _CX;
};


struct CURVE_FITTING_COST_CX_3MIX
{
    CURVE_FITTING_COST_CX_3MIX(double A, double E, double R, double V, double alpha, double CX):_A(A), _E(E),
                        _R(R), _1_V(1 / V), _alpha(alpha), _CX(CX)
                       {}
    
    // 残差计算
    template<typename T>
    bool operator()(
        const T *const CX_p, T* residual
    ) const{
        // CL - fL
        /*
            CL_p_mix3 = (CX_0, 
                    CX_alpha, CX_alpha2, 
                    CX_A, CX_A2, 
                    CX_E, CX_E2, 
                    CX_R, CX_R2, 
                    CX_1_V, CX_1_V2, 
                    CX_AE, CX_AR, CX_A_1_V, CX_Aalpha, 
                    CX_ER, CX_E_1_V, CX_Ealpha, 
                    CX_R_1_V, CX_Ralpha, 
                    CX_1_Valpha,
                    CX_alpha3, CX_A3, CX_E3, CX_R3, CX_1_V3)^T
        */
        residual[0] = T(_CX) - CX_p[0] - 
                      CX_p[1] * _alpha - CX_p[2] * pow(_alpha, 2) -
                      CX_p[3] * _A - CX_p[4] * pow (_A, 2) - 
                      CX_p[5] * _E - CX_p[6] * pow (_E, 2) -
                      CX_p[7] * _R - CX_p[8] * pow (_R, 2) -
                      CX_p[9] * _1_V - CX_p[10] * pow (_1_V, 2) -
                      CX_p[11] * _A * _E - CX_p[12] * _A * _R - CX_p[13] * _A * _1_V - CX_p[14] * _A * _alpha -
                      CX_p[15] * _E * _R - CX_p[16] * _E * _1_V - CX_p[17] * _E * _alpha -
                      CX_p[18] * _R * _1_V - CX_p[19] * _R * _alpha -
                      CX_p[20] * _1_V * _alpha - 
                      CX_p[21] * pow(_alpha, 3) - CX_p[22] * pow(_A, 3) - CX_p[23] * pow(_E, 3) - CX_p[24] * pow(_R, 3) - CX_p[25] * pow(_1_V, 3);
        return true;
    }
    const double _A, _E, _R, _1_V, _alpha, _CX;
};


class AcroParamters
{
private:

    enum CX{
        CL=0u,
        CD=1u,
        Cl=2u,
        Cm=3u,
        Cn=4u
    };

    std::string src_path_;
    std::vector<Error> error;
    std::ifstream infile;
    FixWingParam param;

    /*
        CX_p = CX_alpha0, CX_alpha, CX_alpha2, CXs_alpha3,
            CX_A0, CX_A, CX_A, CX_A2, CX_A3, 
            CX_E0, CX_E, CX_E2, CX_E3, 
            CX_R0, CX_R, CX_R2, CX_R3, 
            CX_1_V0, CX_1_V, CX_1_V2, CX_1_V3)^T
    */
    Eigen::VectorXd CL_;
    Eigen::VectorXd CD_;
    Eigen::VectorXd Cl_;
    Eigen::VectorXd Cm_;
    Eigen::VectorXd Cn_;

    /*
        CL_p = (CL_0, 
                CL_alpha, CL_alpha2 / 2, CL_alpha3 / 6,
                CL_A, CL_A2 / 2, CL_A3 / 6, 
                CL_E, CL_E2 / 2, CL_E3 / 6, 
                CL_R, CL_R2 / 2, CL_R3 / 6, 
                CL_1_V, CL_1_V2 / 2, CL_1_V3 / 6)^T
     */
    Eigen::VectorXd CL_p;

    /*
        CD_p = (CD_0, 
                CD_alpha, CD_alpha2 / 2, CD_alpha3 / 6,
                CD_A, CD_A2 / 2, CD_A3 / 6, 
                CD_E, CD_E2 / 2, CD_E3 / 6, 
                CD_R, CD_R2 / 2, CD_R3 / 6, 
                CD_1_V, CD_1_V2 / 2, CD_1_V3 / 6)^T
    */
    Eigen::VectorXd CD_p;

    /*
        Cl_p = (Cl_0, 
                Cl_alpha, Cl_alpha2 / 2, Cl_alpha3 / 6,
                Cl_A, Cl_A2 / 2, Cl_A3 / 6, 
                Cl_E, Cl_E2 / 2, Cl_E3 / 6, 
                Cl_R, Cl_R2 / 2, Cl_R3 / 6, 
                Cl_1_V, Cl_1_V2 / 2, Cl_1_V3 / 6)^T
    */
    Eigen::VectorXd Cl_p;

    /*
        Cm_p = (Cm_0, 
                Cm_alpha, Cm_alpha2 / 2, Cm_alpha3 / 6,
                Cm_A, Cm_A2 / 2, Cm_A3 / 6, 
                Cm_E, Cm_E2 / 2, Cm_E3 / 6, 
                Cm_R, Cm_R2 / 2, Cm_R3 / 6, 
                Cm_1_V, Cm_1_V2 / 2, Cm_1_V3 / 6)^T
    */
    Eigen::VectorXd Cm_p;
    
    /*
        Cn_p = (Cn_0, 
                Cn_alpha, Cn_alpha2 / 2, Cn_alpha3 / 6,
                Cn_A, Cn_A2 / 2, Cn_A3 / 6, 
                Cn_E, Cn_E2 / 2, Cn_E3 / 6, 
                Cn_R, Cn_R2 / 2, Cn_R3 / 6, 
                Cn_1_V, Cn_1_V2 / 2, Cn_1_V3 / 6)^T
    */
    Eigen::VectorXd Cn_p;

    /*
        CL_p_mix2 = (CL_0, 
                CL_alpha, CL_alpha2, 
                CL_A, CL_A2, 
                CL_E, CL_E2, 
                CL_R, CL_R2, 
                CL_V, CL_V2, 
                CL_AE, CL_AR, CL_AV, CL_Aalpha, 
                CL_ER, CL_EV, CL_Ealpha, 
                CL_RV, CL_Ralpha, 
                CL_Valpha)^T
    */
    Eigen::VectorXd CL_p_mix2;

    /*
        CD_p_mix2 = (CD_0, 
                CD_alpha, CD_alpha2, 
                CD_A, CD_A2, 
                CD_E, CD_E2, 
                CD_R, CD_R2, 
                CD_V, CD_V2, 
                CD_AE, CD_AR, CD_AV, CD_Aalpha, 
                CD_ER, CD_EV, CD_Ealpha, 
                CD_RV, CD_Ralpha, 
                CD_Valpha)^T
    */
    Eigen::VectorXd CD_p_mix2;

    /*
        Cl_p_mix2 = (Cl_0, 
                Cl_alpha, Cl_alpha2, 
                Cl_A, Cl_A2, 
                Cl_E, Cl_E2, 
                Cl_R, Cl_R2, 
                Cl_V, Cl_V2, 
                Cl_AE, Cl_AR, Cl_AV, Cl_Aalpha, 
                Cl_ER, Cl_EV, Cl_Ealpha, 
                Cl_RV, Cl_Ralpha, 
                Cl_Valpha)^T
    */
    Eigen::VectorXd Cl_p_mix2;

    /*
        Cm_p_mix2 = (Cl_0, 
                Cl_alpha, Cl_alpha2, 
                Cl_A, Cl_A2, 
                Cl_E, Cl_E2, 
                Cl_R, Cl_R2, 
                Cl_V, Cl_V2, 
                Cl_AE, Cl_AR, Cl_AV, Cl_Aalpha, 
                Cl_ER, Cl_EV, Cl_Ealpha, 
                Cl_RV, Cl_Ralpha, 
                Cl_Valpha)^T
    */
    Eigen::VectorXd Cm_p_mix2;

    /*
        Cn_p_mix2 = (Cl_0, 
                Cl_alpha, Cl_alpha2, 
                Cl_A, Cl_A2, 
                Cl_E, Cl_E2, 
                Cl_R, Cl_R2, 
                Cl_V, Cl_V2, 
                Cl_AE, Cl_AR, Cl_AV, Cl_Aalpha, 
                Cl_ER, Cl_EV, Cl_Ealpha, 
                Cl_RV, Cl_Ralpha, 
                Cl_Valpha)^T
    */
    Eigen::VectorXd Cn_p_mix2;


    /*
        CL_p_mix3 = (CL_0, 
                CL_alpha, CL_alpha2, 
                CL_A, CL_A2, 
                CL_E, CL_E2, 
                CL_R, CL_R2, 
                CL_V, CL_V2, 
                CL_AE, CL_AR, CL_AV, CL_Aalpha, 
                CL_ER, CL_EV, CL_Ealpha, 
                CL_RV, CL_Ralpha, 
                CL_Valpha,
                CL_alpha3, CL_A3, CL_E3, CL_R3, CL_V3)^T
    */
    Eigen::VectorXd CL_p_mix3;

    /*
        CD_p_mix3 = (CD_0, 
                CD_alpha, CD_alpha2, 
                CD_A, CD_A2, 
                CD_E, CD_E2, 
                CD_R, CD_R2, 
                CD_V, CD_V2, 
                CD_AE, CD_AR, CD_AV, CD_Aalpha, 
                CD_ER, CD_EV, CD_Ealpha, 
                CD_RV, CD_Ralpha, 
                CD_Valpha,
                CD_alpha3, CD_A3, CD_E3, CD_R3, CD_V3)^T
    */
    Eigen::VectorXd CD_p_mix3;

    /*
        Cl_p_mix3 = (CD_0, 
                Cl_alpha, CD_alpha2, 
                Cl_A, Cl_A2, 
                Cl_E, Cl_E2, 
                Cl_R, Cl_R2, 
                Cl_V, Cl_V2, 
                Cl_AE, Cl_AR, Cl_AV, Cl_Aalpha, 
                Cl_ER, Cl_EV, Cl_Ealpha, 
                Cl_RV, Cl_Ralpha, 
                Cl_Valpha,
                Cl_alpha3, Cl_A3, Cl_E3, Cl_R3, Cl_V3)^T
    */
    Eigen::VectorXd Cl_p_mix3;

    /*
        Cm_p_mix3 = (Cm_0, 
                Cm_alpha, Cm_alpha2, 
                Cm_A, Cm_A2, 
                Cm_E, Cm_E2, 
                Cm_R, Cm_R2, 
                Cm_V, Cm_V2, 
                Cm_AE, Cm_AR, Cm_AV, Cm_Aalpha, 
                Cm_ER, Cm_EV, Cm_Ealpha, 
                Cm_RV, Cm_Ralpha, 
                Cm_Valpha,
                Cm_alpha3, Cm_A3, Cm_E3, Cm_R3, Cm_V3)^T
    */
    Eigen::VectorXd Cm_p_mix3;

    /*
        Cn_p_mix3 = (Cn_0, 
                Cn_alpha, Cn_alpha2, 
                Cn_A, Cn_A2, 
                Cn_E, Cn_E2, 
                Cn_R, Cn_R2, 
                Cn_V, Cn_V2, 
                Cn_AE, Cn_AR, Cn_AV, Cn_Aalpha, 
                Cn_ER, Cn_EV, Cn_Ealpha, 
                Cn_RV, Cn_Ralpha, 
                Cn_Valpha,
                Cn_alpha3, Cn_A3, Cn_E3, Cn_R3, Cn_V3)^T
    */
    Eigen::VectorXd Cn_p_mix3;

    ceres::Problem CL_problem;
    ceres::Problem CD_problem;
    ceres::Problem Cl_problem;
    ceres::Problem Cm_problem;
    ceres::Problem Cn_problem;

    ceres::Problem problem_CL;
    ceres::Problem problem_CD;
    ceres::Problem problem_Cl;
    ceres::Problem problem_Cm;
    ceres::Problem problem_Cn;


    ceres::Problem problem_mix2_CL;
    ceres::Problem problem_mix2_CD;
    ceres::Problem problem_mix2_Cl;
    ceres::Problem problem_mix2_Cm;
    ceres::Problem problem_mix2_Cn;

    
    ceres::Problem problem_mix3_CL;
    ceres::Problem problem_mix3_CD;
    ceres::Problem problem_mix3_Cl;
    ceres::Problem problem_mix3_Cm;
    ceres::Problem problem_mix3_Cn;
public:
    AcroParamters()
    {
        CL_.resize(20, 1);
        CD_.resize(20, 1);
        Cl_.resize(20, 1);
        Cm_.resize(20, 1);
        Cn_.resize(20, 1);

        CL_p.resize(16, 1);
        CD_p.resize(16, 1);
        Cl_p.resize(16, 1);
        Cm_p.resize(16, 1);
        Cn_p.resize(16, 1);
        
        CL_p_mix2.resize(21, 1);
        CD_p_mix2.resize(21, 1);
        Cl_p_mix2.resize(21, 1);
        Cm_p_mix2.resize(21, 1);
        Cn_p_mix2.resize(21, 1);

        CL_p_mix3.resize(26, 1);
        CD_p_mix3.resize(26, 1);
        Cl_p_mix3.resize(26, 1);
        Cm_p_mix3.resize(26, 1);
        Cn_p_mix3.resize(26, 1);
    };
    bool readParamters(std::string src_path);
    void ceresCurveFitting();
    /*
     * A, E, R, V, alpha   副翼 升降舵 方向 空速 迎角
     */
    AcroDynamicParamters getAcroParam(double A, double E, double R, double V, double alpha);
    AcroDynamicParamters getAcroParam_mix2(double A, double E, double R, double V, double alpha);
    AcroDynamicParamters getAcroParam_mix3(double A, double E, double R, double V, double alpha);
    AcroDynamicParamters getAcroParam_CX(double A, double E, double R, double V, double alpha);

    void CXplot(CX _CX);

    ~AcroParamters();
};

AcroParamters::~AcroParamters()
{
}

bool AcroParamters::readParamters(std::string src_path)
{
    src_path_ = src_path;
    uint32_t sum = 0;
    for(uint32_t i = 0; i < 3; i++)
    {
        std::string A;
        switch (i)
        {
            case 0:
                A = "A-20_";
                break;
            case 1:
                A = "A0_";
                break;
            case 2:
                A = "A20_";
                break;
        }
        for(uint32_t j = 0; j < 3; j++)
        {
            std::string E;
            switch (j)
            {
                case 0:
                    E = "E-20_";
                    break;
                case 1:
                    E = "E0_";
                    break;
                case 2:
                    E = "E20_";
                    break;
            }
            for(uint32_t k = 0; k < 3; k++)
            {
                std::string R;
                switch (k)
                {
                    case 0:
                        R = "R-20";
                        break;
                    case 1:
                        R = "R0";
                        break;
                    case 2:
                        R = "R20";
                        break;
                }
                for(uint32_t f = 0; f < 5; f++)
                {
                    std::string V;
                    switch (f)
                    {
                        case 0:
                            V = "V10_";
                            break;
                        case 1:
                            V = "V15_";
                            break;
                        case 2:
                            V = "V20_";
                            break;
                        case 3:
                            V = "V25_";
                            break;
                        case 4:
                            V = "V30_";
                            break;
                    }
                    std::string name = src_path_ + A + E + R + "/" + V + A + E + R + ".txt";
                    std::cout << name << std::endl;
                    infile.open(name);
                    std::string str;
                    if (infile)
                    {
                        for(int i = 1; i <= 8; ++i)
                        {
                            getline(infile, str);
                            std::cout << str << '\n';
                        }
                    }
                    uint32_t count = 0;
                    if (infile)
                    {
                        while (!infile.eof())
                        {
                            if(count >= 91)
                                break;
                            infile >> param.paramters[i][j][k][f][count].alpha >> param.paramters[i][j][k][f][count].Beta >> param.paramters[i][j][k][f][count].CL >> param.paramters[i][j][k][f][count].CDi >> param.paramters[i][j][k][f][count].CDv >> param.paramters[i][j][k][f][count].CD >> param.paramters[i][j][k][f][count].CY >> param.paramters[i][j][k][f][count].Cl >> param.paramters[i][j][k][f][count].Cm >> param.paramters[i][j][k][f][count].Cn >> param.paramters[i][j][k][f][count].Cni >> param.paramters[i][j][k][f][count].QInf >> param.paramters[i][j][k][f][count].XCP;
                            std::cout << std::setw(10) << param.paramters[i][j][k][f][count].alpha << ", " << std::setw(10) << param.paramters[i][j][k][f][count].Beta << ", " << std::setw(10) << param.paramters[i][j][k][f][count].CL << ", " << std::setw(10) << param.paramters[i][j][k][f][count].CDi << ", " << std::setw(10) << param.paramters[i][j][k][f][count].CDv << ", " << std::setw(10) << param.paramters[i][j][k][f][count].CD << ", " << std::setw(10) << param.paramters[i][j][k][f][count].CY << ", " << std::setw(10) << param.paramters[i][j][k][f][count].Cl << ", " << std::setw(10) << param.paramters[i][j][k][f][count].Cm << ", " << std::setw(10) << param.paramters[i][j][k][f][count].Cn << ", " << std::setw(10) << param.paramters[i][j][k][f][count].Cni << ", " << std::setw(10) << param.paramters[i][j][k][f][count].QInf << ", " << std::setw(10) << param.paramters[i][j][k][f][count].XCP << '\n';
                            count++;
                        }
                        if(count == 91)
                            sum++;
                        else
                        {
                            Error r;
                            r.count = count;
                            r.error = name;
                            error.push_back(r);
                        }
                    }
                    infile.close();
                    // sum++;
                }
            }
        }
    }
    std::cout << sum << '\n';
    for(auto er:error)
    {
        ROS_INFO_STREAM(er.error << '\t' << er.count);
        ROS_BREAK();
        return false;
    }
    return true;
}

AcroDynamicParamters AcroParamters::getAcroParam(double A, double E, double R, double V, double alpha)
{
    double _1_V = 1 / V;
    AcroDynamicParamters param;
    Eigen::VectorXd state(16);
    state << 1, alpha, pow(alpha, 2), pow(alpha, 3),
                A, pow(A, 2), pow(A, 3),
                E, pow(E, 2), pow(E, 3),
                R, pow(R, 2), pow(R, 3),
                _1_V, pow(_1_V, 2), pow(_1_V, 3);

    param.CL = state.transpose() * CL_p;
    param.CD = state.transpose() * CD_p;
    param.Cl = state.transpose() * Cl_p;
    param.Cm = state.transpose() * Cm_p;
    param.Cn = state.transpose() * Cn_p;

    return param;
}

AcroDynamicParamters AcroParamters::getAcroParam_mix2(double A, double E, double R, double V, double alpha)
{
    double _1_V = 1 / V;
    AcroDynamicParamters param_mix;
    Eigen::VectorXd state(21);
    
    state << 1, 
            alpha, pow(alpha, 2), 
            A, pow(A, 2),
            E, pow(E, 2),
            R, pow(R, 2),
            _1_V, pow(_1_V, 2),
            A * E, A * R, A * _1_V, A * alpha,
            E * R, E * _1_V, E * alpha,
            R * _1_V, R * alpha,
            _1_V * alpha;

    param_mix.CL = state.transpose() * CL_p_mix2;
    param_mix.CD = state.transpose() * CD_p_mix2;
    param_mix.Cl = state.transpose() * Cl_p_mix2;
    param_mix.Cm = state.transpose() * Cm_p_mix2;
    param_mix.Cn = state.transpose() * Cn_p_mix2;

    return param_mix;
}

AcroDynamicParamters AcroParamters::getAcroParam_mix3(double A, double E, double R, double V, double alpha)
{
    double _1_V = 1 / V;
    AcroDynamicParamters param_mix;
    Eigen::VectorXd state(26);
    
    state << 1, 
            alpha, pow(alpha, 2), 
            A, pow(A, 2),
            E, pow(E, 2),
            R, pow(R, 2),
            _1_V, pow(_1_V, 2),
            A * E, A * R, A * _1_V, A * alpha,
            E * R, E * _1_V, E * alpha,
            R * _1_V, R * alpha,
            _1_V * alpha,
            pow(alpha, 3), pow(A, 3), pow(E, 3), pow(R, 3), pow(_1_V, 3);

    param_mix.CL = state.transpose() * CL_p_mix3;
    param_mix.CD = state.transpose() * CD_p_mix3;
    param_mix.Cl = state.transpose() * Cl_p_mix3;
    param_mix.Cm = state.transpose() * Cm_p_mix3;
    param_mix.Cn = state.transpose() * Cn_p_mix3;

    return param_mix;
}

AcroDynamicParamters AcroParamters::getAcroParam_CX(double A, double E, double R, double V, double alpha)
{
    double _1_V = 1 / V;
    AcroDynamicParamters param;
    
    // state << 1, alpha, pow(alpha, 2), pow(alpha, 3),
    //          A, pow(A, 2), pow(A, 3),
    //          E, pow(E, 2), pow(E, 3),
    //          R, pow(R, 2), pow(R, 3),
    //          _1_V, pow(_1_V, 2), pow(_1_V, 3);
    // double CL = ( CL_(0) +  CL_(1) * alpha +  CL_(2) * pow(alpha, 2) +  CL_(3) * pow(alpha, 3)) *
    //            ( CL_(4) +  CL_(5) * A     +  CL_(6) * pow(A, 2)     +  CL_(7) * pow(A, 3))     *
    //            ( CL_(8) +  CL_(9) * E     + CL_(10) * pow(E, 2)     + CL_(11) * pow(E, 3))     *
    //            (CL_(12) + CL_(13) * R     + CL_(14) * pow(R, 2)     + CL_(15) * pow(R, 3))     * 
    //            (CL_(16) + CL_(17) * _1_V  + CL_(18) * pow(_1_V, 2)  + CL_(19) * pow(_1_V, 3));
    // double CD = ( CD_(0) +  CD_(1) * alpha +  CD_(2) * pow(alpha, 2) +  CD_(3) * pow(alpha, 3)) *
    //            ( CD_(4) +  CD_(5) * A     +  CD_(6) * pow(A, 2)     +  CD_(7) * pow(A, 3))     *
    //            ( CD_(8) +  CD_(9) * E     + CD_(10) * pow(E, 2)     + CD_(11) * pow(E, 3))     *
    //            (CD_(12) + CD_(13) * R     + CD_(14) * pow(R, 2)     + CD_(15) * pow(R, 3))     * 
    //            (CD_(16) + CD_(17) * _1_V  + CD_(18) * pow(_1_V, 2)  + CD_(19) * pow(_1_V, 3));
    param.CL = ( CL_(0) +  CL_(1) * alpha +  CL_(2) * pow(alpha, 2) +  CL_(3) * pow(alpha, 3)) *
               ( CL_(4) +  CL_(5) * A     +  CL_(6) * pow(A, 2)     +  CL_(7) * pow(A, 3))     *
               ( CL_(8) +  CL_(9) * E     + CL_(10) * pow(E, 2)     + CL_(11) * pow(E, 3))     *
               (CL_(12) + CL_(13) * R     + CL_(14) * pow(R, 2)     + CL_(15) * pow(R, 3))     * 
               (CL_(16) + CL_(17) * _1_V  + CL_(18) * pow(_1_V, 2)  + CL_(19) * pow(_1_V, 3));

    param.CD = ( CD_(0) +  CD_(1) * alpha +  CD_(2) * pow(alpha, 2) +  CD_(3) * pow(alpha, 3)) *
               ( CD_(4) +  CD_(5) * A     +  CD_(6) * pow(A, 2)     +  CD_(7) * pow(A, 3))     *
               ( CD_(8) +  CD_(9) * E     + CD_(10) * pow(E, 2)     + CD_(11) * pow(E, 3))     *
               (CD_(12) + CD_(13) * R     + CD_(14) * pow(R, 2)     + CD_(15) * pow(R, 3))     * 
               (CD_(16) + CD_(17) * _1_V  + CD_(18) * pow(_1_V, 2)  + CD_(19) * pow(_1_V, 3));
    param.Cl = ( Cl_(0) +  Cl_(1) * alpha +  Cl_(2) * pow(alpha, 2) +  Cl_(3) * pow(alpha, 3)) *
               ( Cl_(4) +  Cl_(5) * A     +  Cl_(6) * pow(A, 2)     +  Cl_(7) * pow(A, 3))     *
               ( Cl_(8) +  Cl_(9) * E     + Cl_(10) * pow(E, 2)     + Cl_(11) * pow(E, 3))     *
               (Cl_(12) + Cl_(13) * R     + Cl_(14) * pow(R, 2)     + Cl_(15) * pow(R, 3))     * 
               (Cl_(16) + Cl_(17) * _1_V  + Cl_(18) * pow(_1_V, 2)  + Cl_(19) * pow(_1_V, 3));
    param.Cm = ( Cm_(0) +  Cm_(1) * alpha +  Cm_(2) * pow(alpha, 2) +  Cm_(3) * pow(alpha, 3)) *
               ( Cm_(4) +  Cm_(5) * A     +  Cm_(6) * pow(A, 2)     +  Cm_(7) * pow(A, 3))     *
               ( Cm_(8) +  Cm_(9) * E     + Cm_(10) * pow(E, 2)     + Cm_(11) * pow(E, 3))     *
               (Cm_(12) + Cm_(13) * R     + Cm_(14) * pow(R, 2)     + Cm_(15) * pow(R, 3))     * 
               (Cm_(16) + Cm_(17) * _1_V  + Cm_(18) * pow(_1_V, 2)  + Cm_(19) * pow(_1_V, 3));
    param.Cn = ( Cn_(0) +  Cn_(1) * alpha +  Cn_(2) * pow(alpha, 2) +  Cn_(3) * pow(alpha, 3)) *
               ( Cn_(4) +  Cn_(5) * A     +  Cn_(6) * pow(A, 2)     +  Cn_(7) * pow(A, 3))     *
               ( Cn_(8) +  Cn_(9) * E     + Cn_(10) * pow(E, 2)     + Cn_(11) * pow(E, 3))     *
               (Cn_(12) + Cn_(13) * R     + Cn_(14) * pow(R, 2)     + Cn_(15) * pow(R, 3))     * 
               (Cn_(16) + Cn_(17) * _1_V  + Cn_(18) * pow(_1_V, 2)  + Cn_(19) * pow(_1_V, 3));

    return param;
}

// 使用最小二乘法进行系统辨识
void AcroParamters::ceresCurveFitting()
{
    double _CL_[20] = {0.1, 0.1, 0.1, 0.1, 
                      0.1, 0.1, 0.1, 0.1, 
                      0.1, 0.1, 0.1, 0.1,
                      0.1, 0.1, 0.1, 0.1,
                      0.1, 0.1, 0.1, 0.1};
    double _CD_[20] = {0.2, 0.2, 0.1, 0.1, 
                      0.2, 0.2, 0.1, 0.1, 
                      0.2, 0.2, 0.1, 0.1,
                      0.2, 0.2, 0.1, 0.1,
                      0.2, 0.2, 0.1, 0.1};
    double _Cl_[20] = {0.1, 0.1, 0.1, 0.1, 
                      0.1, 0.1, 0.1, 0.1, 
                      0.1, 0.1, 0.1, 0.1,
                      0.1, 0.1, 0.1, 0.1,
                      0.1, 0.1, 0.1, 0.1};
    double _Cm_[20] = {0.1, 0.1, 0.1, 0.1, 
                      0.1, 0.1, 0.1, 0.1, 
                      0.1, 0.1, 0.1, 0.1,
                      0.1, 0.1, 0.1, 0.1,
                      0.1, 0.1, 0.1, 0.1};
    double _Cn_[20] = {0.1, 0.1, 0.1, 0.1, 
                      0.1, 0.1, 0.1, 0.1, 
                      0.1, 0.1, 0.1, 0.1,
                      0.1, 0.1, 0.1, 0.1,
                      0.1, 0.1, 0.1, 0.1};

    double CL_p_[16] = {0.5, 0.5, 0.0, 0.0, 
                        0.0, 0.0, 0.0, 0.0, 
                        0.0, 0.0, 0.0, 0.0,
                        0.0, 0.0, 0.0, 0.0};
    double CD_p_[16] = {0.5, 0.5, 0.0, 0.0, 
                        0.0, 0.0, 0.0, 0.0, 
                        0.0, 0.0, 0.0, 0.0,
                        0.0, 0.0, 0.0, 0.0};
    double Cl_p_[16] = {0.5, 0.5, 0.0, 0.0, 
                        0.0, 0.0, 0.0, 0.0, 
                        0.0, 0.0, 0.0, 0.0,
                        0.0, 0.0, 0.0, 0.0};
    double Cm_p_[16] = {0.5, 0.5, 0.0, 0.0, 
                        0.0, 0.0, 0.0, 0.0, 
                        0.0, 0.0, 0.0, 0.0,
                        0.0, 0.0, 0.0, 0.0};
    double Cn_p_[16] = {0.5, 0.5, 0.0, 0.0, 
                        0.0, 0.0, 0.0, 0.0, 
                        0.0, 0.0, 0.0, 0.0,
                        0.0, 0.0, 0.0, 0.0};

    double A = -20.0, E = -20.0, R = -20.0, V = -20.0, alpha = -45.0;
    for(int i = 0; i < 3; i++)
    {
        for(int j = 0; j < 3; j++)
        {
            for(int k = 0; k < 3; k++)
            {
                for(int s = 0; s < 5; s++)
                {
                    for(int a = 0; a < 91; a++)
                    {
                        switch (i)
                        {
                            case 0:
                                A = -20.0;
                                break;
                            case 1:
                                A = 0.0;
                                break;
                            case 2:
                                A = 20.0;
                                break;
                        }
                        switch (j)
                        {
                            case 0:
                                E = -20.0;
                                break;
                            case 1:
                                E = 0.0;
                                break;
                            case 2:
                                E = 20.0;
                                break;
                        }
                        switch (k)
                        {
                            case 0:
                                R = -20.0;
                                break;
                            case 1:
                                R = 0.0;
                                break;
                            case 2:
                                R = 20.0;
                                break;
                        }
                        switch (s)
                        {
                            case 0:
                                V = 10.0;
                                break;
                            case 1:
                                V = 15.0;
                                break;
                            case 2:
                                V = 20.0;
                                break;
                            case 3:
                                V = 25.0;
                                break;
                            case 4:
                                V = 30.0;
                                break;
                        }
                        alpha += a;

                        CL_problem.AddResidualBlock(
                            new ceres::AutoDiffCostFunction<CUADC::CURVE_FITTING_COST_CX_, 1, 20>(
                                new CUADC::CURVE_FITTING_COST_CX_(A, E, R, V, alpha, param.paramters[i][j][k][s][a].CL)
                            ),
                            nullptr,
                            _CL_
                        );
                        CD_problem.AddResidualBlock(
                            new ceres::AutoDiffCostFunction<CUADC::CURVE_FITTING_COST_CX_, 1, 20>(
                                new CUADC::CURVE_FITTING_COST_CX_(A, E, R, V, alpha, param.paramters[i][j][k][s][a].CD)
                            ),
                            nullptr,
                            _CD_
                        );
                        Cl_problem.AddResidualBlock(
                            new ceres::AutoDiffCostFunction<CUADC::CURVE_FITTING_COST_CX_, 1, 20>(
                                new CUADC::CURVE_FITTING_COST_CX_(A, E, R, V, alpha, param.paramters[i][j][k][s][a].Cl)
                            ),
                            nullptr,
                            _Cl_
                        );
                        Cm_problem.AddResidualBlock(
                            new ceres::AutoDiffCostFunction<CUADC::CURVE_FITTING_COST_CX_, 1, 20>(
                                new CUADC::CURVE_FITTING_COST_CX_(A, E, R, V, alpha, param.paramters[i][j][k][s][a].Cm)
                            ),
                            nullptr,
                            _Cm_
                        );
                        Cn_problem.AddResidualBlock(
                            new ceres::AutoDiffCostFunction<CUADC::CURVE_FITTING_COST_CX_, 1, 20>(
                                new CUADC::CURVE_FITTING_COST_CX_(A, E, R, V, alpha, param.paramters[i][j][k][s][a].Cn)
                            ),
                            nullptr,
                            _Cn_
                        );

                        problem_CL.AddResidualBlock(
                            new ceres::AutoDiffCostFunction<CUADC::CURVE_FITTING_COST_CX, 1, 16>(
                                new CUADC::CURVE_FITTING_COST_CX(A, E, R, V, alpha, param.paramters[i][j][k][s][a].CL)
                            ),
                            nullptr,
                            CL_p_
                        );
                        problem_CD.AddResidualBlock(
                            new ceres::AutoDiffCostFunction<CUADC::CURVE_FITTING_COST_CX, 1, 16>(
                                new CUADC::CURVE_FITTING_COST_CX(A, E, R, V, alpha, param.paramters[i][j][k][s][a].CD)
                            ),
                            nullptr,
                            CD_p_
                        );
                        problem_Cl.AddResidualBlock(
                            new ceres::AutoDiffCostFunction<CUADC::CURVE_FITTING_COST_CX, 1, 16>(
                                new CUADC::CURVE_FITTING_COST_CX(A, E, R, V, alpha, param.paramters[i][j][k][s][a].Cl)
                            ),
                            nullptr,
                            Cl_p_
                        );
                        problem_Cm.AddResidualBlock(
                            new ceres::AutoDiffCostFunction<CUADC::CURVE_FITTING_COST_CX, 1, 16>(
                                new CUADC::CURVE_FITTING_COST_CX(A, E, R, V, alpha, param.paramters[i][j][k][s][a].Cm)
                            ),
                            nullptr,
                            Cm_p_
                        );
                        problem_Cn.AddResidualBlock(
                            new ceres::AutoDiffCostFunction<CUADC::CURVE_FITTING_COST_CX, 1, 16>(
                                new CUADC::CURVE_FITTING_COST_CX(A, E, R, V, alpha, param.paramters[i][j][k][s][a].Cn)
                            ),
                            nullptr,
                            Cn_p_
                        );
                        alpha = -45.0;
                    }
                }
            }
        }
    }

    ceres::Solver::Options CL_options;
    CL_options.max_num_iterations = 100; 
    CL_options.linear_solver_type = ceres::DENSE_NORMAL_CHOLESKY; 
    CL_options.minimizer_progress_to_stdout = true;
    ceres::Solver::Summary CL_summary;
    ceres::Solve(CL_options, &CL_problem, &CL_summary);  // 开始优化

    ceres::Solver::Options CD_options;
    CD_options.max_num_iterations = 50; 
    CD_options.linear_solver_type = ceres::DENSE_NORMAL_CHOLESKY; 
    CD_options.minimizer_progress_to_stdout = true;
    ceres::Solver::Summary CD_summary;
    ceres::Solve(CD_options, &CD_problem, &CD_summary);  // 开始优化

    ceres::Solver::Options Cl_options;
    Cl_options.max_num_iterations = 100; 
    Cl_options.linear_solver_type = ceres::DENSE_NORMAL_CHOLESKY; 
    Cl_options.minimizer_progress_to_stdout = true;
    ceres::Solver::Summary Cl_summary;
    ceres::Solve(Cl_options, &Cl_problem, &Cl_summary);  // 开始优化

    ceres::Solver::Options Cm_options;
    Cm_options.max_num_iterations = 100; 
    Cm_options.linear_solver_type = ceres::DENSE_NORMAL_CHOLESKY; 
    Cm_options.minimizer_progress_to_stdout = true;
    ceres::Solver::Summary Cm_summary;
    ceres::Solve(Cm_options, &Cm_problem, &Cm_summary);  // 开始优化

    ceres::Solver::Options Cn_options;
    Cn_options.max_num_iterations = 100; 
    Cn_options.linear_solver_type = ceres::DENSE_NORMAL_CHOLESKY; 
    Cn_options.minimizer_progress_to_stdout = true;
    ceres::Solver::Summary Cn_summary;
    ceres::Solve(Cn_options, &Cn_problem, &Cn_summary);  // 开始优化
    



    ceres::Solver::Options options_CL;
    options_CL.max_num_iterations = 50; 
    options_CL.linear_solver_type = ceres::DENSE_NORMAL_CHOLESKY; 
    options_CL.minimizer_progress_to_stdout = true;
    ceres::Solver::Summary summary_CL;
    ceres::Solve(options_CL, &problem_CL, &summary_CL);  // 开始优化

    ceres::Solver::Options options_CD;
    options_CD.max_num_iterations = 50; 
    options_CD.linear_solver_type = ceres::DENSE_NORMAL_CHOLESKY; 
    options_CD.minimizer_progress_to_stdout = true;
    ceres::Solver::Summary summary_CD;
    ceres::Solve(options_CD, &problem_CD, &summary_CD);  // 开始优化
    
    ceres::Solver::Options options_Cl;
    options_Cl.max_num_iterations = 50; 
    options_Cl.linear_solver_type = ceres::DENSE_NORMAL_CHOLESKY; 
    options_Cl.minimizer_progress_to_stdout = true;
    ceres::Solver::Summary summary_Cl;
    ceres::Solve(options_Cl, &problem_Cl, &summary_Cl);  // 开始优化

    ceres::Solver::Options options_Cm;
    options_Cm.max_num_iterations = 50; 
    options_Cm.linear_solver_type = ceres::DENSE_NORMAL_CHOLESKY; 
    options_Cm.minimizer_progress_to_stdout = true;
    ceres::Solver::Summary summary_Cm;
    ceres::Solve(options_Cm, &problem_Cm, &summary_Cm);  // 开始优化

    ceres::Solver::Options options_Cn;
    options_Cn.max_num_iterations = 50; 
    options_Cn.linear_solver_type = ceres::DENSE_NORMAL_CHOLESKY; 
    options_Cn.minimizer_progress_to_stdout = true;
    ceres::Solver::Summary summary_Cn;
    ceres::Solve(options_Cn, &problem_Cn, &summary_Cn);  // 开始优化

    std::cout << "*************CL CD paramters*************" << '\n';
    std::cout << CL_summary.BriefReport() << std::endl;
    std::cout << CD_summary.BriefReport() << std::endl;
    // std::cout << Cl_summary.BriefReport() << std::endl;
    // std::cout << Cm_summary.BriefReport() << std::endl;
    // std::cout << Cn_summary.BriefReport() << std::endl;
    std::cout << "*************CL CD paramters*************" << '\n';

    std::cout << "*************Single paramters*************" << '\n';
    std::cout << summary_CL.BriefReport() << std::endl;
    std::cout << summary_CD.BriefReport() << std::endl;
    std::cout << summary_Cm.BriefReport() << std::endl;
    std::cout << summary_Cl.BriefReport() << std::endl;
    std::cout << summary_Cn.BriefReport() << std::endl;
    std::cout << "*************Single paramters*************" << '\n';

    for(int i = 0; i < 20; ++i)
    {
        this->CL_(i) = _CL_[i];
        this->CD_(i) = _CD_[i];
        this->Cl_(i) = _Cl_[i];
        this->Cm_(i) = _Cm_[i];
        this->Cn_(i) = _Cn_[i];
    }

    std::cout << "CL estimated is : " << '\n';
    std::cout << CL_ << '\n';
    std::cout << "CD estimated is : " << '\n';
    std::cout << CD_ << '\n';
    std::cout << "Cl estimated is : " << '\n';
    std::cout << Cl_ << '\n';
    std::cout << "Cm estimated is : " << '\n';
    std::cout << Cm_ << '\n';
    std::cout << "Cn estimated is : " << '\n';
    std::cout << Cn_ << '\n';

    for(int i = 0; i < 16; ++i)
    {
        CL_p(i) = CL_p_[i];
        CD_p(i) = CD_p_[i];
        Cl_p(i) = Cl_p_[i];
        Cm_p(i) = Cm_p_[i];
        Cn_p(i) = Cn_p_[i];
    }
    std::cout << "Single paramters CL estimated is : " << '\n';
    std::cout << CL_p << '\n';
    std::cout << "Single paramters CD estimated is : " << '\n';
    std::cout << CD_p << '\n';
    std::cout << "Single paramters Cl estimated is : " << '\n';
    std::cout << Cl_p << '\n';
    std::cout << "Single paramters Cm estimated is : " << '\n';
    std::cout << Cm_p << '\n';
    std::cout << "Single paramters Cn estimated is : " << '\n';
    std::cout << Cn_p << '\n';


    double CL_p_2mix_[21] = {CL_p[0], 
                            CL_p[1], CL_p[2], 
                            CL_p[4], CL_p[5],
                            CL_p[7], CL_p[8], 
                            CL_p[10], CL_p[11], 
                            CL_p[12], CL_p[13], 
                            0.0, 0.0, 0.0, 0.0, 
                            0.0, 0.0, 0.0, 
                            0.0, 0.0,
                            0.0};
    double CD_p_2mix_[21] = {CD_p[0], 
                            CD_p[1], CD_p[2], 
                            CD_p[4], CD_p[5],
                            CD_p[7], CD_p[8], 
                            CD_p[10], CD_p[11], 
                            CD_p[12], CD_p[13], 
                            0.0, 0.0, 0.0, 0.0, 
                            0.0, 0.0, 0.0, 
                            0.0, 0.0,
                            0.0};
    double Cl_p_2mix_[21] = {Cl_p[0], 
                            Cl_p[1], Cl_p[2], 
                            Cl_p[4], Cl_p[5],
                            Cl_p[7], Cl_p[8], 
                            Cl_p[10], Cl_p[11], 
                            Cl_p[12], Cl_p[13], 
                            0.0, 0.0, 0.0, 0.0, 
                            0.0, 0.0, 0.0, 
                            0.0, 0.0,
                            0.0};
    double Cm_p_2mix_[21] = {Cm_p[0], 
                            Cm_p[1], Cm_p[2], 
                            Cm_p[4], Cm_p[5],
                            Cm_p[7], Cm_p[8], 
                            Cm_p[10], Cm_p[11], 
                            Cm_p[12], Cm_p[13], 
                            0.0, 0.0, 0.0, 0.0, 
                            0.0, 0.0, 0.0, 
                            0.0, 0.0,
                            0.0};
    double Cn_p_2mix_[21] = {Cn_p[0], 
                            Cn_p[1], Cn_p[2], 
                            Cn_p[4], Cn_p[5],
                            Cn_p[7], Cn_p[8], 
                            Cn_p[10], Cn_p[11], 
                            Cn_p[12], Cn_p[13], 
                            0.0, 0.0, 0.0, 0.0, 
                            0.0, 0.0, 0.0, 
                            0.0, 0.0,
                            0.0};

    double CL_p_3mix_[26] = {CL_p[0], 
                        CL_p[1], CL_p[2], 
                        CL_p[4], CL_p[5],
                        CL_p[7], CL_p[8], 
                        CL_p[10], CL_p[11], 
                        CL_p[12], CL_p[13], 
                        0.0, 0.0, 0.0, 0.0, 
                        0.0, 0.0, 0.0, 
                        0.0, 0.0,
                        0.0,
                        CL_p[3], CL_p[6], CL_p[9], CL_p[12], CL_p[15]};
    double CD_p_3mix_[26] = {CD_p[0], 
                        CD_p[1], CD_p[2], 
                        CD_p[4], CD_p[5],
                        CD_p[7], CD_p[8], 
                        CD_p[10], CD_p[11], 
                        CD_p[12], CD_p[13], 
                        0.0, 0.0, 0.0, 0.0, 
                        0.0, 0.0, 0.0, 
                        0.0, 0.0,
                        0.0,
                        CD_p[3], CD_p[6], CD_p[9], CD_p[12], CD_p[15]};
    double Cl_p_3mix_[26] = {Cl_p[0], 
                        Cl_p[1], Cl_p[2], 
                        Cl_p[4], Cl_p[5],
                        Cl_p[7], Cl_p[8], 
                        Cl_p[10], Cl_p[11], 
                        Cl_p[12], Cl_p[13], 
                        0.0, 0.0, 0.0, 0.0, 
                        0.0, 0.0, 0.0, 
                        0.0, 0.0,
                        0.0,
                        Cl_p[3], Cl_p[6], Cl_p[9], Cl_p[12], Cl_p[15]};
    double Cm_p_3mix_[26] = {Cm_p[0], 
                        Cm_p[1], Cm_p[2], 
                        Cm_p[4], Cm_p[5],
                        Cm_p[7], Cm_p[8], 
                        Cm_p[10], Cm_p[11], 
                        Cm_p[12], Cm_p[13], 
                        0.0, 0.0, 0.0, 0.0, 
                        0.0, 0.0, 0.0, 
                        0.0, 0.0,
                        0.0,
                        Cm_p[3], Cm_p[6], Cm_p[9], Cm_p[12], Cm_p[15]};
    double Cn_p_3mix_[26] = {Cn_p[0], 
                        Cn_p[1], Cn_p[2], 
                        Cn_p[4], Cn_p[5],
                        Cn_p[7], Cn_p[8], 
                        Cn_p[10], Cn_p[11], 
                        Cn_p[12], Cn_p[13], 
                        0.0, 0.0, 0.0, 0.0, 
                        0.0, 0.0, 0.0, 
                        0.0, 0.0,
                        0.0,
                        Cn_p[3], Cn_p[6], Cn_p[9], Cn_p[12], Cn_p[15]};

    for(int i = 0; i < 3; i++)
    {
        for(int j = 0; j < 3; j++)
        {
            for(int k = 0; k < 3; k++)
            {
                for(int s = 0; s < 5; s++)
                {
                    for(int a = 0; a < 91; a++)
                    {
                        switch (i)
                        {
                            case 0:
                                A = -20.0;
                                break;
                            case 1:
                                A = 0.0;
                                break;
                            case 2:
                                A = 20.0;
                                break;
                        }
                        switch (j)
                        {
                            case 0:
                                E = -20.0;
                                break;
                            case 1:
                                E = 0.0;
                                break;
                            case 2:
                                E = 20.0;
                                break;
                        }
                        switch (k)
                        {
                            case 0:
                                R = -20.0;
                                break;
                            case 1:
                                R = 0.0;
                                break;
                            case 2:
                                R = 20.0;
                                break;
                        }
                        switch (s)
                        {
                            case 0:
                                V = 10.0;
                                break;
                            case 1:
                                V = 15.0;
                                break;
                            case 2:
                                V = 20.0;
                                break;
                            case 3:
                                V = 25.0;
                                break;
                            case 4:
                                V = 30.0;
                                break;
                        }
                        alpha += a;

                        //mix2
                        problem_mix2_CL.AddResidualBlock(
                            new ceres::AutoDiffCostFunction<CUADC::CURVE_FITTING_COST_CX_2MIX, 1, 21>(
                                new CUADC::CURVE_FITTING_COST_CX_2MIX(A, E, R, V, alpha, param.paramters[i][j][k][s][a].CL)
                            ),
                            nullptr,
                            CL_p_2mix_
                        );
                        problem_mix2_CD.AddResidualBlock(
                            new ceres::AutoDiffCostFunction<CUADC::CURVE_FITTING_COST_CX_2MIX, 1, 21>(
                                new CUADC::CURVE_FITTING_COST_CX_2MIX(A, E, R, V, alpha, param.paramters[i][j][k][s][a].CD)
                            ),
                            nullptr,
                            CD_p_2mix_
                        );
                        problem_mix2_Cl.AddResidualBlock(
                            new ceres::AutoDiffCostFunction<CUADC::CURVE_FITTING_COST_CX_2MIX, 1, 21>(
                                new CUADC::CURVE_FITTING_COST_CX_2MIX(A, E, R, V, alpha, param.paramters[i][j][k][s][a].Cl)
                            ),
                            nullptr,
                            Cl_p_2mix_
                        );
                        problem_mix2_Cm.AddResidualBlock(
                            new ceres::AutoDiffCostFunction<CUADC::CURVE_FITTING_COST_CX_2MIX, 1, 21>(
                                new CUADC::CURVE_FITTING_COST_CX_2MIX(A, E, R, V, alpha, param.paramters[i][j][k][s][a].Cm)
                            ),
                            nullptr,
                            Cm_p_2mix_
                        );
                        problem_mix2_Cn.AddResidualBlock(
                            new ceres::AutoDiffCostFunction<CUADC::CURVE_FITTING_COST_CX_2MIX, 1, 21>(
                                new CUADC::CURVE_FITTING_COST_CX_2MIX(A, E, R, V, alpha, param.paramters[i][j][k][s][a].Cn)
                            ),
                            nullptr,
                            Cn_p_2mix_
                        );

                        // mix3
                        problem_mix3_CL.AddResidualBlock(
                            new ceres::AutoDiffCostFunction<CUADC::CURVE_FITTING_COST_CX_3MIX, 1, 26>(
                                new CUADC::CURVE_FITTING_COST_CX_3MIX(A, E, R, V, alpha, param.paramters[i][j][k][s][a].CL)
                            ),
                            nullptr,
                            CL_p_3mix_
                        );
                        problem_mix3_CD.AddResidualBlock(
                            new ceres::AutoDiffCostFunction<CUADC::CURVE_FITTING_COST_CX_3MIX, 1, 26>(
                                new CUADC::CURVE_FITTING_COST_CX_3MIX(A, E, R, V, alpha, param.paramters[i][j][k][s][a].CD)
                            ),
                            nullptr,
                            CD_p_3mix_
                        );
                        problem_mix3_Cl.AddResidualBlock(
                            new ceres::AutoDiffCostFunction<CUADC::CURVE_FITTING_COST_CX_3MIX, 1, 26>(
                                new CUADC::CURVE_FITTING_COST_CX_3MIX(A, E, R, V, alpha, param.paramters[i][j][k][s][a].Cl)
                            ),
                            nullptr,
                            Cl_p_3mix_
                        );
                        problem_mix3_Cm.AddResidualBlock(
                            new ceres::AutoDiffCostFunction<CUADC::CURVE_FITTING_COST_CX_3MIX, 1, 26>(
                                new CUADC::CURVE_FITTING_COST_CX_3MIX(A, E, R, V, alpha, param.paramters[i][j][k][s][a].Cm)
                            ),
                            nullptr,
                            Cm_p_3mix_
                        );
                        problem_mix3_Cn.AddResidualBlock(
                            new ceres::AutoDiffCostFunction<CUADC::CURVE_FITTING_COST_CX_3MIX, 1, 26>(
                                new CUADC::CURVE_FITTING_COST_CX_3MIX(A, E, R, V, alpha, param.paramters[i][j][k][s][a].Cn)
                            ),
                            nullptr,
                            Cn_p_3mix_
                        );
                        
                        alpha = -45.0;
                    }
                }
            }
        }
    }

    //mix2

    ceres::Solver::Options options_mix2_CL;
    options_mix2_CL.max_num_iterations = 50;
    options_mix2_CL.linear_solver_type = ceres::DENSE_NORMAL_CHOLESKY;
    options_mix2_CL.minimizer_progress_to_stdout = true;
    ceres::Solver::Summary summary_mix2_CL;
    ceres::Solve(options_mix2_CL, &problem_mix2_CL, &summary_mix2_CL);  // 开始优化

    ceres::Solver::Options options_mix2_CD;
    options_mix2_CD.max_num_iterations = 50;
    options_mix2_CD.linear_solver_type = ceres::DENSE_NORMAL_CHOLESKY;
    options_mix2_CD.minimizer_progress_to_stdout = true;
    ceres::Solver::Summary summary_mix2_CD;
    ceres::Solve(options_mix2_CD, &problem_mix2_CD, &summary_mix2_CD);  // 开始优化

    ceres::Solver::Options options_mix2_Cl;
    options_mix2_Cl.max_num_iterations = 50;
    options_mix2_Cl.linear_solver_type = ceres::DENSE_NORMAL_CHOLESKY;
    options_mix2_Cl.minimizer_progress_to_stdout = true;
    ceres::Solver::Summary summary_mix2_Cl;
    ceres::Solve(options_mix2_Cl, &problem_mix2_Cl, &summary_mix2_Cl);  // 开始优化

    ceres::Solver::Options options_mix2_Cm;
    options_mix2_Cm.max_num_iterations = 50;
    options_mix2_Cm.linear_solver_type = ceres::DENSE_NORMAL_CHOLESKY;
    options_mix2_Cm.minimizer_progress_to_stdout = true;
    ceres::Solver::Summary summary_mix2_Cm;
    ceres::Solve(options_mix2_Cm, &problem_mix2_Cm, &summary_mix2_Cm);  // 开始优化

    ceres::Solver::Options options_mix2_Cn;
    options_mix2_Cn.max_num_iterations = 50;
    options_mix2_Cn.linear_solver_type = ceres::DENSE_NORMAL_CHOLESKY;
    options_mix2_Cn.minimizer_progress_to_stdout = true;
    ceres::Solver::Summary summary_mix2_Cn;
    ceres::Solve(options_mix2_Cn, &problem_mix2_Cn, &summary_mix2_Cn);  // 开始优化

    //mix3

    ceres::Solver::Options options_mix3_CL;
    options_mix3_CL.max_num_iterations = 50;
    options_mix3_CL.linear_solver_type = ceres::DENSE_NORMAL_CHOLESKY;
    options_mix3_CL.minimizer_progress_to_stdout = true;
    ceres::Solver::Summary summary_mix3_CL;
    ceres::Solve(options_mix3_CL, &problem_mix3_CL, &summary_mix3_CL);  // 开始优化
    
    ceres::Solver::Options options_mix3_CD;
    options_mix3_CD.max_num_iterations = 50;
    options_mix3_CD.linear_solver_type = ceres::DENSE_NORMAL_CHOLESKY;
    options_mix3_CD.minimizer_progress_to_stdout = true;
    ceres::Solver::Summary summary_mix3_CD;
    ceres::Solve(options_mix3_CD, &problem_mix3_CD, &summary_mix3_CD);  // 开始优化

    ceres::Solver::Options options_mix3_Cl;
    options_mix3_Cl.max_num_iterations = 50;
    options_mix3_Cl.linear_solver_type = ceres::DENSE_NORMAL_CHOLESKY;
    options_mix3_Cl.minimizer_progress_to_stdout = true;
    ceres::Solver::Summary summary_mix3_Cl;
    ceres::Solve(options_mix3_Cl, &problem_mix3_Cl, &summary_mix3_Cl);  // 开始优化

    ceres::Solver::Options options_mix3_Cm;
    options_mix3_Cm.max_num_iterations = 50;
    options_mix3_Cm.linear_solver_type = ceres::DENSE_NORMAL_CHOLESKY;
    options_mix3_Cm.minimizer_progress_to_stdout = true;
    ceres::Solver::Summary summary_mix3_Cm;
    ceres::Solve(options_mix3_Cm, &problem_mix3_Cm, &summary_mix3_Cm);  // 开始优化

    ceres::Solver::Options options_mix3_Cn;
    options_mix3_Cn.max_num_iterations = 50;
    options_mix3_Cn.linear_solver_type = ceres::DENSE_NORMAL_CHOLESKY;
    options_mix3_Cn.minimizer_progress_to_stdout = true;
    ceres::Solver::Summary summary_mix3_Cn;
    ceres::Solve(options_mix3_Cn, &problem_mix3_Cn, &summary_mix3_Cn);  // 开始优化

    std::cout << "***************Mix2 paramters***************" << '\n';
    std::cout << summary_mix2_CL.BriefReport() << std::endl;
    std::cout << summary_mix2_CD.BriefReport() << std::endl;
    std::cout << summary_mix2_Cl.BriefReport() << std::endl;
    std::cout << summary_mix2_Cm.BriefReport() << std::endl;
    std::cout << summary_mix2_Cn.BriefReport() << std::endl;
    std::cout << "***************Mix2 paramters***************" << '\n';

    std::cout << "***************Mix3 paramters***************" << '\n';
    std::cout << summary_mix3_CL.BriefReport() << std::endl;
    std::cout << summary_mix3_CD.BriefReport() << std::endl;
    std::cout << summary_mix3_Cl.BriefReport() << std::endl;
    std::cout << summary_mix3_Cm.BriefReport() << std::endl;
    std::cout << summary_mix3_Cn.BriefReport() << std::endl;
    std::cout << "***************Mix3 paramters***************" << '\n';

    for(int i = 0; i < 21; ++i)
    {
        CL_p_mix2(i) = CL_p_2mix_[i];
        CD_p_mix2(i) = CD_p_2mix_[i];
        Cl_p_mix2(i) = Cl_p_2mix_[i];
        Cm_p_mix2(i) = Cm_p_2mix_[i];
        Cn_p_mix2(i) = Cn_p_2mix_[i];
    }
    std::cout << "Mix2 paramters estimated is : " << '\n';
    std::cout << CL_p_mix2 << '\n';
    std::cout << "Mix2 paramters estimated is : " << '\n';
    std::cout << CD_p_mix2 << '\n';
    std::cout << "Mix2 paramters estimated is : " << '\n';
    std::cout << Cl_p_mix2 << '\n';
    std::cout << "Mix2 paramters estimated is : " << '\n';
    std::cout << Cm_p_mix2 << '\n';
    std::cout << "Mix2 paramters estimated is : " << '\n';
    std::cout << Cn_p_mix2 << '\n';
    
    for(int i = 0; i < 26; ++i)
    {
        CL_p_mix3(i) = CL_p_3mix_[i];
        CD_p_mix3(i) = CD_p_3mix_[i];
        Cl_p_mix3(i) = Cl_p_3mix_[i];
        Cm_p_mix3(i) = Cm_p_3mix_[i];
        Cn_p_mix3(i) = Cn_p_3mix_[i];
    }
    std::cout << "Mix3 paramters estimated is : " << '\n';
    std::cout << CL_p_mix3 << '\n';
    std::cout << "Mix3 paramters estimated is : " << '\n';
    std::cout << CD_p_mix3 << '\n';
    std::cout << "Mix3 paramters estimated is : " << '\n';
    std::cout << Cl_p_mix3 << '\n';
    std::cout << "Mix3 paramters estimated is : " << '\n';
    std::cout << Cm_p_mix3 << '\n';
    std::cout << "Mix3 paramters estimated is : " << '\n';
    std::cout << Cn_p_mix3 << '\n';

    int n = 15;

    std::cout  << "(A:-20, E:0, R:-20, V:10, alpha:20)" << '\n' ;
    std::cout << "Sing : " <<  getAcroParam(-20.0, 0, -20.0, 10, 0).CL << std::setw(n) << getAcroParam(-20.0, 0, -20.0, 10, 0).CD << std::setw(n) << getAcroParam(-20.0, 0, -20.0, 10, 0).Cl << std::setw(n) << getAcroParam(-20.0, 0, -20.0, 10, 0).Cm << std::setw(n) << getAcroParam(-20.0, 0, -20.0, 10, 0).Cn << '\n';
    std::cout << "Mix2 : " <<  getAcroParam_mix2(-20.0, 0, -20.0, 10, 0).CL << std::setw(n) << getAcroParam_mix2(-20.0, 0, -20.0, 10, 0).CD << std::setw(n) << getAcroParam_mix2(-20.0, 0, -20.0, 10, 0).Cl << std::setw(n) << getAcroParam_mix2(-20.0, 0, -20.0, 10, 0).Cm << std::setw(n) << getAcroParam_mix2(-20.0, 0, -20.0, 10, 0).Cn << '\n';
    std::cout << "Mix3 : " <<  getAcroParam_mix3(-20.0, 0, -20.0, 10, 0).CL << std::setw(n) << getAcroParam_mix3(-20.0, 0, -20.0, 10, 0).CD << std::setw(n) << getAcroParam_mix3(-20.0, 0, -20.0, 10, 0).Cl << std::setw(n) << getAcroParam_mix3(-20.0, 0, -20.0, 10, 0).Cm << std::setw(n) << getAcroParam_mix3(-20.0, 0, -20.0, 10, 0).Cn << '\n';
    std::cout << "CX_  : " << getAcroParam_CX(-20.0, 0, -20.0, 10, 0).CL << std::setw(n) << getAcroParam_CX(-20.0, 0, -20.0, 10, 0).CD << std::setw(n) << getAcroParam_CX(-20.0, 0, -20.0, 10, 0).Cl << std::setw(n) << getAcroParam_CX(-20.0, 0, -20.0, 10, 0).Cm << std::setw(n) << getAcroParam_CX(-20.0, 0, -20.0, 10, 0).Cn << std::endl;
    // std::cout << "CX_  : " << getAcroParam_CX(-20.0, 0, -20.0, 10, 0).CL << std::setw(n) << getAcroParam_CX(-20.0, 0, -20.0, 10, 0).CD << std::endl;
    std::cout << "Truth: " << param.paramters[0][1][0][0][45].CL << std::setw(n) << param.paramters[0][1][0][0][45].CD << std::setw(n) << param.paramters[0][1][0][0][45].Cl << std::setw(n) << param.paramters[0][1][0][0][45].Cm << std::setw(n) << param.paramters[0][1][0][0][45].Cn << std::endl;
    CX cx = CX::CL;
    CXplot(cx);
}

void AcroParamters::CXplot(CX _CX)
{   
    pangolin::DataLog log_CL;
    pangolin::DataLog log_CD;
    pangolin::DataLog log_Cl;
    pangolin::DataLog log_Cm;
    pangolin::DataLog log_Cn;


    switch (_CX)
    {
        case CX::CL:
        {
            pangolin::CreateWindowAndBind("CL", 640*2, 480*2);
            std::vector<std::string> labels_CL;
            labels_CL.push_back(std::string("Estimate_CL"));
            labels_CL.push_back(std::string("Single Paramter Estimate_CL"));
            labels_CL.push_back(std::string("Mix2 Paramter Estimate_CL"));
            labels_CL.push_back(std::string("Mix3 Paramter Estimate_CL"));
            labels_CL.push_back(std::string("Esti_Error"));
            labels_CL.push_back(std::string("Single_Error"));
            labels_CL.push_back(std::string("Error_MIX2"));
            labels_CL.push_back(std::string("Error_MIX3"));
            labels_CL.push_back(std::string("Truth_CL"));
            log_CL.SetLabels(labels_CL);
            static pangolin::Plotter plotter_CL(&log_CL, -5.0f, 5.0f,-5.0f,5.0f, 0.1, 0.1);
            plotter_CL.SetBounds(0.0, 1.0, 0.0, 1.0);
            pangolin::Display("CL").AddDisplay(plotter_CL);
            break;
        }
        case CX::CD:
        {
            pangolin::CreateWindowAndBind("CD", 640*2, 480*2);
            std::vector<std::string> labels_CD;
            labels_CD.push_back(std::string("Estimate_CD"));
            labels_CD.push_back(std::string("Single Paramter Estimate_CD"));
            labels_CD.push_back(std::string("Mix2 Paramter Estimate_CD"));
            labels_CD.push_back(std::string("Mix3 Paramter Estimate_CD"));
            labels_CD.push_back(std::string("Esti_Error"));
            labels_CD.push_back(std::string("Error"));
            labels_CD.push_back(std::string("Error_MIX2"));
            labels_CD.push_back(std::string("Error_MIX3"));
            labels_CD.push_back(std::string("Truth_CD"));
            log_CD.SetLabels(labels_CD);
            static pangolin::Plotter plotter_CD(&log_CD, -2.0f, 2.0f,-2.0f,2.0f, 0.01, 0.01);
            plotter_CD.SetBounds(0.0, 1.0, 0.0, 1.0);
            pangolin::Display("CD").AddDisplay(plotter_CD);
            break;
        }
        case CX::Cl:
        { 
            pangolin::CreateWindowAndBind("Cl", 640*2, 480*2);
            std::vector<std::string> labels_Cl;
            labels_Cl.push_back(std::string("Estimate_Cl"));
            labels_Cl.push_back(std::string("Single Paramter Estimate_Cl"));
            labels_Cl.push_back(std::string("Mix2 Paramter Estimate_Cl"));
            labels_Cl.push_back(std::string("Mix3 Paramter Estimate_Cl"));
            labels_Cl.push_back(std::string("Esti_Error"));
            labels_Cl.push_back(std::string("Error"));
            labels_Cl.push_back(std::string("Error_MIX2"));
            labels_Cl.push_back(std::string("Error_MIX3"));
            labels_Cl.push_back(std::string("Truth_Cl"));
            log_Cl.SetLabels(labels_Cl);
            static pangolin::Plotter plotter_Cl(&log_Cl, -2.0f, 2.0f,-2.0f,2.0f, 0.01, 0.01);
            plotter_Cl.SetBounds(0.0, 1.0, 0.0, 1.0);
            pangolin::Display("Cl").AddDisplay(plotter_Cl);
            break;
        }
        case CX::Cm:
        { 
            pangolin::CreateWindowAndBind("Cm", 640*2, 480*2);
            std::vector<std::string> labels_Cm;
            labels_Cm.push_back(std::string("Estimate_Cm"));
            labels_Cm.push_back(std::string("Single Paramter Estimate_Cm"));
            labels_Cm.push_back(std::string("Mix2 Paramter Estimate_Cm"));
            labels_Cm.push_back(std::string("Mix3 Paramter Estimate_Cm"));
            labels_Cm.push_back(std::string("Esti_Error"));
            labels_Cm.push_back(std::string("Error"));
            labels_Cm.push_back(std::string("Error_MIX2"));
            labels_Cm.push_back(std::string("Error_MIX3"));
            labels_Cm.push_back(std::string("Truth_Cm"));
            log_Cm.SetLabels(labels_Cm);
            static pangolin::Plotter plotter_Cm(&log_Cm, -2.0f, 2.0f,-2.0f,2.0f, 0.01, 0.01);
            plotter_Cm.SetBounds(0.0, 1.0, 0.0, 1.0);
            pangolin::Display("Cm").AddDisplay(plotter_Cm);
            break;
        }
        case CX::Cn:
        {
            pangolin::CreateWindowAndBind("Cn", 640*2, 480*2);
            std::vector<std::string> labels_Cn;
            labels_Cn.push_back(std::string("Estimate_Cn"));
            labels_Cn.push_back(std::string("Single Paramter Estimate_Cn"));
            labels_Cn.push_back(std::string("Mix2 Paramter Estimate_Cn"));
            labels_Cn.push_back(std::string("Mix3 Paramter Estimate_Cn"));
            labels_Cn.push_back(std::string("Esti_Error"));
            labels_Cn.push_back(std::string("Error"));
            labels_Cn.push_back(std::string("Error_MIX2"));
            labels_Cn.push_back(std::string("Error_MIX3"));
            labels_Cn.push_back(std::string("Truth_Cn"));
            log_Cn.SetLabels(labels_Cn);
            static pangolin::Plotter plotter_Cn(&log_Cn, -2.0f, 2.0f,-2.0f,2.0f, 0.01, 0.01);
            plotter_Cn.SetBounds(0.0, 1.0, 0.0, 1.0);
            pangolin::Display("Cn").AddDisplay(plotter_Cn);
            break;
        }
    }



    int alpha = -45;

    AcroDynamicParamters truth_param, esti_param, sing_param;
    AcroDynamicParamters esi_param_mix2, esi_param_mix3;
    double est_Error_CL = 0, Error_CL = 0, Error_mix2_CL = 0, Error_mix3_CL = 0;
    double est_Error_CD = 0, Error_CD = 0, Error_mix2_CD = 0, Error_mix3_CD = 0;
    double est_Error_Cl = 0, Error_Cl = 0, Error_mix2_Cl = 0, Error_mix3_Cl = 0;
    double est_Error_Cm = 0, Error_Cm = 0, Error_mix2_Cm = 0, Error_mix3_Cm = 0;
    double est_Error_Cn = 0, Error_Cn = 0, Error_mix2_Cn = 0, Error_mix3_Cn = 0;
    while( !pangolin::ShouldQuit())
    {
        if(alpha > 45)
            alpha = -45;
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

        truth_param    = param.paramters[1][1][1][0][alpha+45];
        esti_param     =   getAcroParam_CX(0, 0, 0, 10, alpha);
        sing_param     =      getAcroParam(0, 0, 0, 10, alpha);
        esi_param_mix2 = getAcroParam_mix2(0, 0, 0, 10, alpha);
        esi_param_mix3 = getAcroParam_mix3(0, 0, 0, 10, alpha);

        est_Error_CL  = pow(    esti_param.CL - truth_param.CL, 2);
        Error_CL      = pow(    sing_param.CL - truth_param.CL, 2);
        Error_mix2_CL = pow(esi_param_mix2.CL - truth_param.CL, 2);
        Error_mix3_CL = pow(esi_param_mix3.CL - truth_param.CL, 2);

        est_Error_CD  = pow(    esti_param.CD - truth_param.CD, 2);
        Error_CD      = pow(    sing_param.CD - truth_param.CD, 2);
        Error_mix2_CD = pow(esi_param_mix2.CD - truth_param.CD, 2);
        Error_mix3_CD = pow(esi_param_mix3.CD - truth_param.CD, 2);

        est_Error_Cl  = pow(    esti_param.Cl - truth_param.Cl, 2);
        Error_Cl      = pow(    sing_param.Cl - truth_param.Cl, 2);
        Error_mix2_Cl = pow(esi_param_mix2.Cl - truth_param.Cl, 2);
        Error_mix3_Cl = pow(esi_param_mix3.Cl - truth_param.Cl, 2);

        est_Error_Cm  = pow(    esti_param.Cm - truth_param.Cm, 2);
        Error_Cm      = pow(    sing_param.Cm - truth_param.Cm, 2);
        Error_mix2_Cm = pow(esi_param_mix2.Cm - truth_param.Cm, 2);
        Error_mix3_Cm = pow(esi_param_mix3.Cm - truth_param.Cm, 2);

        est_Error_Cn  = pow(    esti_param.Cn - truth_param.Cn, 2);
        Error_Cn      = pow(    sing_param.Cn - truth_param.Cn, 2);
        Error_mix2_Cn = pow(esi_param_mix2.Cn - truth_param.Cn, 2);
        Error_mix3_Cn = pow(esi_param_mix3.Cn - truth_param.Cn, 2);

        switch (_CX)
        {
            case CX::CL:
            {
                log_CL.Log(esti_param.CL, sing_param.CL, esi_param_mix2.CL, esi_param_mix3.CL, 0, 0, 0, 0, truth_param.CL);
                break;
            }
            case CX::CD:
            {
                log_CD.Log(esti_param.CD, sing_param.CD, esi_param_mix2.CD, esi_param_mix3.CD, 0, 0, 0, 0, truth_param.CD);
                break;
            }
            case CX::Cl:
            {
                log_Cl.Log(esti_param.Cl, sing_param.Cl, esi_param_mix2.Cl, esi_param_mix3.Cl, 0, 0, 0, 0, truth_param.Cl);
                break;
            }
            case CX::Cm:
            {
                log_Cm.Log(esti_param.Cm, sing_param.Cm, esi_param_mix2.Cm, esi_param_mix3.Cm, 0, 0, 0, 0, truth_param.Cm);
                break;
            }
            case CX::Cn:
            {
                log_Cn.Log(esti_param.Cn, sing_param.Cn, esi_param_mix2.Cn, esi_param_mix3.Cn, 0, 0, 0, 0, truth_param.Cn);

                break;
            }
        }
        alpha++;
        pangolin::FinishFrame();
    }
    
}

} // namespace CUADC
