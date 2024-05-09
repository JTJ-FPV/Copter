#include <ros/ros.h>
#include <fstream>
#include <string>
#include <iostream>
#include <vector>

using namespace std;

struct AcroDynamicParamters
{
    double alpha, Beta, CL, CDi, CDv, CD, CY, Cl, Cm, Cn, Cni, QInf, XCP;
};

struct FixWingParam
{
    // 空速 迎角 升降舵 副翼 方向
    // AcroDynamic paramters[30][41][31][31][21];  

    // 副翼 升降舵 方向 空速 迎角
    AcroDynamicParamters paramters[3][3][3][5][91];  
};

struct Error
{
    std::string error;
    int count;
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "readfile");

	std::ifstream infile;

    FixWingParam param;

    vector<Error> error;

    // double alpha, Beta, CL, CDi, CDv, CD, CY, Cl, Cm, Cn, Cni, QInf, XCP;
    std::string src = "/home/jtj/CUADC/src/planner/config/fixedwingDynamic/CFD_results/";
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
                    std::string name = src + A + E + R + "/" + V + A + E + R + ".txt";
                    std::cout << name << std::endl;
                    infile.open(name);
                    string str;
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
                            cout << setw(10) << param.paramters[i][j][k][f][count].alpha << ", " << setw(10) << param.paramters[i][j][k][f][count].Beta << ", " << setw(10) << param.paramters[i][j][k][f][count].CL << ", " << setw(10) << param.paramters[i][j][k][f][count].CDi << ", " << setw(10) << param.paramters[i][j][k][f][count].CDv << ", " << setw(10) << param.paramters[i][j][k][f][count].CD << ", " << setw(10) << param.paramters[i][j][k][f][count].CY << ", " << setw(10) << param.paramters[i][j][k][f][count].Cl << ", " << setw(10) << param.paramters[i][j][k][f][count].Cm << ", " << setw(10) << param.paramters[i][j][k][f][count].Cn << ", " << setw(10) << param.paramters[i][j][k][f][count].Cni << ", " << setw(10) << param.paramters[i][j][k][f][count].QInf << ", " << setw(10) << param.paramters[i][j][k][f][count].XCP << '\n';
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
        ROS_INFO_STREAM(er.error << '\t' << er.count);
    // for(int i = 0; i < 91; ++i)
    // {
    //     std::cout << param.paramters[0][0][0][0][i].alpha << '\t' << param.paramters[1][0][1][2][i].Beta << '\t' << param.paramters[1][0][1][2][i].CL << '\t' << param.paramters[1][0][1][2][i].CDi << '\t' << param.paramters[1][0][1][2][i].CDv << '\t' << param.paramters[1][0][1][2][i].CD << '\t' << param.paramters[1][0][1][2][i].CY << '\t' << param.paramters[1][0][1][2][i].Cl << '\t' << param.paramters[1][0][1][2][i].Cm << '\t' << param.paramters[1][0][1][2][i].Cn << '\t' << param.paramters[1][0][1][2][i].Cni << '\t' << param.paramters[1][0][1][2][i].QInf << '\t' << param.paramters[1][0][1][2][i].XCP << '\n';
    // }
    cout << param.paramters[0][0][0][1][90].alpha << '\t' << param.paramters[0][0][0][1][90].Cm << '\n';
	return 0;
}

