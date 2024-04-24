#include "balance.h"
#include "stdint.h"

/**
 * @brief 根据状态反馈计算当前腿长,查表获得LQR的反馈增益,并列式计算LQR的输出
 * @note 得到的腿部力矩输出还要经过综合运动控制系统补偿后映射为两个关节电机输出
 *
 */
void CalcLQR(LinkNPodParam *p, ChassisParam *chassis)
{
    // float k[12][3] = {89.582677,-172.311264,-12.862450,-7.406183,-29.225173,0.351843,13.770239,-11.389716,-18.835586,15.547229,-22.241056,-13.241076,131.619584,-117.579480,32.974720,6.898540,-6.543172,2.125882,37.724640,-38.801531,18.576409,1.358879,-1.070109,2.140319,74.091942,-66.518020,18.803614,51.675325,-46.798148,13.962229,-60.271354,47.855082,121.051548,-3.379791,2.755658,3.836537};
    float k[12][3] ={64.243416,-141.662945,-5.238953,-8.148398,-24.389682,0.397634,21.395779,-17.860637,-16.418171,19.602603,-25.417339,-11.014433,142.594766,-133.925864,43.351448,3.491561,-3.378176,1.773110,
                    8.917718,-15.413525,17.330217,
                    -1.999192,2.228384,1.997761,
                    66.103340,-63.080745,20.024508,
                    43.248993,-41.927122,14.479306,
                    -105.800667,88.004336,111.781567,
                    -4.368323,3.901658,1.945933};
    // float k[12][3] = {59.311625,-138.608031,-6.684034,
    //                     -7.783778,-24.538466,0.602671,
    //                     20.045497,-17.995535,-16.064736,
    //                     19.171193,-26.252811,-10.231878,
    //                     124.777394,-124.218950,39.743263,
    //                     4.519079,-4.794350,1.365983,
    //                     18.127182,-23.915497,21.397369,
    //                     -1.956629,3.113024,2.322263,
    //                     67.251724,-68.189140,23.200587,
    //                     41.934607,-43.278346,16.331815,
    //                     -75.859432,66.058219,115.221597,
    //                     -2.182224,1.821434,2.691092,};
    float T[2] = {0}; // 0 T_wheel  1 T_hip
    float l = p->leg_len;
    float lsqr = l * l;

    // float dist_limit = abs(chassis->target_dist - chassis->dist) > MAX_DIST_TRACK ? sign(chassis->target_dist - chassis->dist) * MAX_DIST_TRACK : (chassis->target_dist - chassis->dist); // todo设置值
    // float vel_limit = abs(chassis->target_v - chassis->vel) > MAX_VEL_TRACK ? sign(chassis->target_v - chassis->vel) * MAX_VEL_TRACK : (chassis->target_v - chassis->vel);


    // 离地检测
    if (p->normal_force < MIN_FLY_F/3 || p->jump_state == RETRACT_LEG)
    {
        for (size_t i = 0; i < 12; i++)
        {
            if(i != 6 && i != 7)
            {
                for (size_t j = 0; j < 3; j++)
                {
                    k[i][j] = 0;
                }
            }
        }
    }
    

    // 计算增益
    for (uint8_t i = 0; i < 2; ++i)
    {
        uint8_t j = i * 6;
        T[i] = (k[j + 0][0] * lsqr + k[j + 0][1] * l + k[j + 0][2]) * -(p->theta) +
            (k[j + 1][0] * lsqr + k[j + 1][1] * l + k[j + 1][2]) * -(p->theta_w) +
            (k[j + 2][0] * lsqr + k[j + 2][1] * l + k[j + 2][2]) * (chassis->target_dist - chassis->dist) +
            (k[j + 3][0] * lsqr + k[j + 3][1] * l + k[j + 3][2]) * (chassis->target_v - chassis->vel) +
            (k[j + 4][0] * lsqr + k[j + 4][1] * l + k[j + 4][2]) * -(chassis->pitch) +
            (k[j + 5][0] * lsqr + k[j + 5][1] * l + k[j + 5][2]) * -(chassis->pitch_w);
    }
    p->T_wheel = T[0];
    p->T_hip = T[1];
}