#include "image.h"

int f[10 * CAMERA_H];//考察连通域联通性
//每个白条子属性
typedef struct {
    uint8_t   left;//左边界
    uint8_t   right;//右边界
    int   connect_num;//连通标记
}range;

//每行的所有白条子
typedef struct {
    uint8_t   num;//每行白条数量
    range   area[white_num_MAX];//该行各白条区域
}all_range;

//属于赛道的每个白条子属性
typedef struct {
    uint8_t   left;//左边界
    uint8_t   right;//右边界
    uint8_t   width;//宽度
}road_range;

//每行属于赛道的每个白条子
typedef struct {
    uint8_t   white_num;
    road_range   connected[white_num_MAX];
}road;

all_range white_range[CAMERA_H];//所有白条子
road my_road[CAMERA_H];//赛道
uint8_t IMG[CAMERA_H][CAMERA_W];//二值化后图像数组
uint8_t left_line[CAMERA_H], right_line[CAMERA_H];//赛道的左右边界
uint8_t mid_line[CAMERA_H];
int all_connect_num = 0;//所有白条子数
uint8_t top_road;//赛道最高处所在行数
int foresight = 65;//前瞻初值
//int TRUE_TH = 210;//外部可调用阈值
uint8_t threshold = 210;//阈值
uint8_t *fullBuffer = NULL;

uint8_t line_type = 0;//赛道类型
int wide_line[CAMERA_W]; //赛道宽度
uint8_t pio_x[4] = { 0 }, pio_y[4] = { 0 };//四点坐标
int any;
uint8_t zebra_flag;
uint8_t zebra_flag2;
uint8_t leftline_change[CAMERA_H] = { 0 };

////////////////////////////////////////////
//功能：二值化
//输入：灰度图片
//输出：二值化图片
//备注：
///////////////////////////////////////////
void THRE()
{
    uint8_t* map;
    uint8_t* my_map;
    map = fullBuffer;
    for (int i = 0; i < 120; i++)
    {
        my_map = &IMG[i][0];
        for (int j = 0; j < 188; j++)
        {
            if ((*map) > threshold)
                (*my_map) = 1;
            else (*my_map) = 0;
            map++;
            my_map++;
        }
    }
}

////////////////////////////////////////////
//功能：粗犷的清车头
//输入：
//输出：
//备注：要根据自己车头的大小进行修改
///////////////////////////////////////////
void head_clear(void)
{
    uint8_t* my_map;
    for (int i = 119; i >= 84; i--)
    {
        my_map = &IMG[i][0];
        for (int j = 40; j <= 135; j++)
        {
            *(my_map+j) = white;
        }
    }
}

////////////////////////////////////////////
//功能：查找父节点
//输入：节点编号
//输出：最老祖先
//备注：含路径压缩
///////////////////////////////////////////
int find_f(int node)
{
    if (f[node] == node)return node;//找到最古老祖先，return
    f[node] = find_f(f[node]);//向上寻找自己的父节点
    return f[node];
}

////////////////////////////////////////////
//功能：提取跳变沿 并对全部白条子标号
//输入：IMG[120][188]
//输出：white_range[120]
//备注：指针提速
///////////////////////////////////////////
void search_white_range()
{
    uint8_t i, j;
    int istart = NEAR_LINE;//处理起始行
    int iend = FAR_LINE;//处理终止行
    int tnum = 0;//当前行白条数
    all_connect_num = 0;//白条编号初始化
    uint8_t* map = NULL;
    for (i = istart; i >= iend; i--)
    {
        map = &IMG[i][LEFT_SIDE];//指针行走加快访问速度
        tnum = 0;
        for (j = LEFT_SIDE; j <= RIGHT_SIDE; j++, map++)
        {
            if ((*map))//遇白条左边界
            {
                tnum++;
                if (tnum >= white_num_MAX)break;
                range* now_white = &white_range[i].area[tnum];
                now_white->left = j;

                //开始向后一个一个像素点找这个白条右边界
                map++;
                j++;

                while ((*map) && j <= RIGHT_SIDE)
                {
                    map++;
                    j++;
                }
                now_white->right = j - 1;
                now_white->connect_num = ++all_connect_num;//白条数加一，给这个白条编号
            }
        }
        white_range[i].num = tnum;
    }
}

////////////////////////////////////////////
//功能：寻找白条子连通性，将全部联通白条子的节点编号刷成最古老祖先的节点编号
//输入：
//输出：
//备注：
///////////////////////////////////////////
void find_all_connect()
{
    //f数组初始化
    for (int i = 1; i <= all_connect_num; i++)
        f[i] = i;

    //u为up d为down 即为当前处理的这两行中的上面那行和下面那行
    //u_num：上面行白条数
    //u_left：上面行当前白条左边界
    //u_right：上面行当前白条右边界
    //i_u：当前处理的这个白条是当前这行（上面行）白条中的第i_u个
    int u_num, i_u, u_left, u_right;
    int d_num, i_d, d_left, d_right;
    all_range* u_white = NULL;
    all_range* d_white = NULL;
    for (int i = NEAR_LINE; i > FAR_LINE; i--)//因为每两行每两行比较 所以循环到FAR_LINE+1
    {
        u_num = white_range[i - 1].num;
        d_num = white_range[i].num;
        u_white = &white_range[i - 1];
        d_white = &white_range[i];
        i_u = 1; i_d = 1;

        //循环到当前行或上面行白条子数耗尽为止
        while (i_u <= u_num && i_d <= d_num)
        {
            //变量先保存，避免下面访问写的冗杂且访问效率低
            u_left = u_white->area[i_u].left;
            u_right = u_white->area[i_u].right;
            d_left = d_white->area[i_d].left;
            d_right = d_white->area[i_d].right;

            if (u_left <= d_right && u_right >= d_left)//如果两个白条联通
                f[find_f(u_white->area[i_u].connect_num)] = find_f(d_white->area[i_d].connect_num);//父节点连起来

            //当前算法规则，手推一下你就知道为啥这样了
            if (d_right > u_right)i_u++;
            if (d_right < u_right)i_d++;
            if (d_right == u_right) { i_u++; i_d++; }
        }
    }
}

////////////////////////////////////////////
//功能：寻找赛道
//输入：
//输出：
//备注：
///////////////////////////////////////////
void find_road()
{
    uint8_t istart = NEAR_LINE;
    uint8_t iend = FAR_LINE;
    top_road = NEAR_LINE;//赛道最高处所在行数，先初始化话为最低处
    int road_f = -1;//赛道所在连通域父节点编号，先初始化为-1，以判断是否找到赛道
    int while_range_num = 0, roud_while_range_num = 0;
    all_range* twhite_range = NULL;
    road* tmy_road = NULL;
    //寻找赛道所在连通域
    // 寻找最中心的白条子
    for (int i = 1; i <= white_range[istart].num; i++)
        if (white_range[istart].area[i].left <= CAMERA_W / 2
            && white_range[istart].area[i].right >= CAMERA_W / 2 && (white_range[istart].area[i].right - white_range[istart].area[i].left) >= 90)
            road_f = find_f(white_range[istart].area[i].connect_num);

    if (road_f == -1)//若赛道没在中间，在113行选一行最长的认为这就是赛道
    {
        int widthmax = 0, jselect = 1;
        for (int i = 1; i <= white_range[istart].num; i++)
            if (white_range[istart].area[i].right - white_range[istart].area[i].left > widthmax)
            {
                widthmax = white_range[istart].area[i].right - white_range[istart].area[i].left;
                jselect = i;
            }
        road_f = find_f(white_range[istart].area[jselect].connect_num);
    }

    //现在我们已经得到了赛道所在连通域父节点编号，接下来把所有父节点编号是road_f的所有白条子扔进赛道数组就行了
    for (int i = istart; i >= iend; i--)
    {
        //变量保存，避免之后写的冗杂且低效
        twhite_range = &white_range[i];
        tmy_road = &my_road[i];
        while_range_num = twhite_range->num;
        tmy_road->white_num = 0;
        roud_while_range_num = 0;
        for (int j = 1; j <= while_range_num; j++)
        {
            if (find_f(twhite_range->area[j].connect_num) == road_f)
            {
                top_road = i;
                tmy_road->white_num++; roud_while_range_num++;
                tmy_road->connected[roud_while_range_num].left = twhite_range->area[j].left;
                tmy_road->connected[roud_while_range_num].right = twhite_range->area[j].right;
                tmy_road->connected[roud_while_range_num].width = twhite_range->area[j].right - twhite_range->area[j].left;

            }
        }
    }
}

////////////////////////////////////////////
//功能：返回相连下一行白条子编号
//输入：i_start起始行  j_start白条标号
//输出：白条标号
//备注：认为下一行与本行赛道重叠部分对多的白条为选定赛道
///////////////////////////////////////////
uint8_t find_continue(uint8_t i_start, uint8_t j_start)
{
    uint8_t j_return;
    uint8_t j;
    uint8_t width_max = 0;
    uint8_t width_new = 0;
    uint8_t left = 0;
    uint8_t right = 0;
    uint8_t dright, dleft, uright, uleft;
    j_return = MISS;//如果没找到，输出255
    if (j_start > my_road[i_start].white_num)
        return MISS;
    //选一个重叠最大的
    for (j = 1; j <= my_road[i_start - 1].white_num; j++)
    {
        dleft = my_road[i_start].connected[j_start].left;
        dright = my_road[i_start].connected[j_start].right;
        uleft = my_road[i_start - 1].connected[j].left;
        uright = my_road[i_start - 1].connected[j].right;
        if (//相连
            dleft < uright
            &&
            dright > uleft
            )
        {
            //计算重叠大小
            if (dleft < uleft) left = uleft;
            else left = dleft;

            if (dright > uright) right = uright;
            else right = dright;

            width_new = right - left + 1;

            if (width_new > width_max)
            {
                width_max = width_new;
                j_return = j;
            }
        }

    }
    return j_return;
}

////////////////////////////////////////////
//功能：通用决定双边
//输入：
//输出：
//备注：
///////////////////////////////////////////
void ordinary_two_line(void)
{
    uint8_t i;
    uint8_t j;
    uint8_t j_continue[CAMERA_H];//第一条连通路径
    uint8_t i_start;
    uint8_t i_end;
    uint8_t j_start = MISS;
    int width_max;

    //寻找起始行最宽的白条子
    i_start = NEAR_LINE;
    i_end = FAR_LINE;
    width_max = 0;
    for (j = 1; j <= my_road[i_start].white_num; j++)
    {
        if (my_road[i_start].connected[j].width > width_max)
        {
            width_max = my_road[i_start].connected[j].width;
            j_start = j;
        }
    }
    j_continue[i_start] = j_start;

    //记录连贯区域编号
    for (i = i_start; i > i_end; i--)
    {
        //如果相连编号大于该行白条数，非正常，从此之后都MISS
        if (j_continue[i] > my_road[i].white_num)
        {
            j_continue[i - 1] = MISS;
        }
        else
        {
            j_continue[i - 1] = find_continue(i, j_continue[i]);
        }

    }

    //全部初始化为MISS
    my_memset(left_line, MISS, CAMERA_H);
    my_memset(right_line, MISS, CAMERA_H);


    for (i = i_start; i > i_end; i--)
    {
        if (j_continue[i] <= my_road[i].white_num)
        {
            left_line[i] = my_road[i].connected[j_continue[i]].left;
            right_line[i] = my_road[i].connected[j_continue[i]].right;
            IMG[i][left_line[i]] = blue;
            IMG[i][right_line[i]] = red;
        }
        else
        {
            left_line[i] = MISS;
            right_line[i] = MISS;
        }
    }
}

////////////////////////////////////////////
//功能：数组初始化
//输入：uint8_t* ptr 数组首地址, uint8_t num初始化的值, uint8_t size数组大小
//输出：
//备注：因为k66库中认为memset函数不安全，所以无法使用；因此需要自己写一个my_memset
///////////////////////////////////////////
void my_memset(uint8_t* ptr, uint8_t num, uint8_t size)
{
    uint8_t* p = ptr;
    uint8_t my_num = num;
    uint8_t Size = size;
    for (int i = 0; i < Size; i++, p++)
    {
        *p = my_num;
    }
}
////////////////////////////////////////////
//功能：中线合成
//输入：左右边界
//输出：中线
//备注：
///////////////////////////////////////////
void get_mid_line(void)
{
    my_memset(mid_line, MISS, CAMERA_H);
    for(int i = NEAR_LINE;i >= FAR_LINE;i--)
        if (left_line[i] != MISS)
        {
            mid_line[i] = (left_line[i] + right_line[i]) / 2;
            if (i < NEAR_LINE)
            {
                any = i;
                midline_check();
            }
        }
        else
        {
            mid_line[i] = MISS;
        }

}
////////////////////////////////////////////
//功能：图像处理主程序
//输入：
//输出：
//备注：
///////////////////////////////////////////
//void image_main()
//{
//    head_clear();
//    search_white_range();
//    find_all_connect();
//    find_road();
//    /*到此处为止，我们已经得到了属于赛道的结构体数组my_road[CAMERA_H]*/
//    ordinary_two_line();
//    get_mid_line();
//
//    for (int i = NEAR_LINE; i >= FAR_LINE; i--)
//        if (mid_line[i] != MISS)
//            IMG[i][mid_line[i]] = red;
//}
float get_error()
{
    float a =  80 - mid_line[foresight];
    if(mid_line[foresight]==MISS)
    {
            a=0;
    }
    return a;
}

void type_line(void)
{
    int a_line_type_1 = 0, a_line_type_2 = 0, a = 0, b = 0, stop = 0, L = 0, R = 0, i = 0;//a类赛道判据1和2
    for (i = 75; i > 30; i--)
    {
        if ((left_line[i] != 255) && (left_line[i - 7] != 255) && (right_line[i] != 255) && (right_line[i - 7] != 255))
        {
            if ((((int)left_line[i] - (int)left_line[i - 7]) >= 5) || (left_line[i] == 0))
            {
                a = 1;//左边界突左

            }
            if ((((int)right_line[i - 7] - (int)right_line[i]) >= 5) || (right_line[i] == 187))
            {
                b = 1;//左边界突右
                //printf("i=%di-7=%d\n", i, i-7);//向右
            }
            if ((a == 1) && (b == 1))//左右边界跳变判据
            {
                line_type = 1;
                i = 20;
            }
        }
    }
    if ((a == 1) && (b != 1))
    {
        printf("c33%d%d\n", a, b);//向左
    }
    if ((a != 1) && (b == 1))
    {
        printf("d44%d%d\n", a, b);//向右
    }
    printf("a%db%d\n",a,b);
    for (i = 75; i > 20; i--)
    {
        if (wide_line[i] >= 160)
        {
            line_type = 1;
            printf("a_1%d\n", line_type);
            i = 20;
        }
    }
    for (i = 110; i > 60; i--)
    {
        if (wide_line[i] > 180)
        {
            line_type = 2;
            i = 60;
        }
    }
}
//////////////////////////////////////////
//功能：进入十字边界寻找
//输入：
//输出：
//备注：
/////////////////////////////////////////
void cross_two_line(void)
{
    int a=0, b=0;
    for (int i = 1; i < 100; i++)
    {
        for (int j = 94; j > 10; j--)
        {
            if ((IMG[i][j] == 0) && (IMG[i][j + 1] == 1) && (IMG[i][j + 5] == 1)&&(IMG[i][j-2] == 0))
            {
                left_line[i] = j;
            j=10;//找到第一个左边界
            }
        }
        //找到第一个边界就跳出循环
        for (int k = 94; k < 170; k++)
        {
            if ((IMG[i][k] == 0) && (IMG[i][k - 1] == 1) && (IMG[i][k - 5] == 1)&&(IMG[i][k + 5] == 0))
            {
                right_line[i] = k;
                k = 170;
            }
        }
        /*if (a != b)
        {
            i = 100;
        }*/
        IMG[i][left_line[i]] = blue;
        IMG[i][right_line[i]] = red;
        wide_line[i] = right_line[i] - left_line[i];//求赛道宽
    }
}
//////////////////////////////////////////
//功能：进入十字边界寻找
//输入：
//输出：
//备注：
/////////////////////////////////////////
void cross_two_line2(void)
{
    int i = 0, j = 0, k = 94,k_ave=0,k_min=0,g=0;
    for (i = 110; i > 10; i--)
    {
        for (j = k; j > 10; j--)
        {
            if ((IMG[i][j] == 0) && (IMG[i][j + 1] == 1) && (IMG[i][j + 5] == 1) && (IMG[i][j - 2] == 0))
            {
                left_line[i] = j;
                j = 1;//找到第一个左边界
            }
        }
        for (j = k; j < 170; j++)
        {
            if ((IMG[i][j] == 0) && (IMG[i][j - 1] == 1) && (IMG[i][j - 5] == 1) && (IMG[i][j + 5] == 0))
            {
                right_line[i] = j;
                j = 180;
            }
        }
        k = (left_line[i] + right_line[i]) / 2;
        if ((i < 105)&&(g==0))
        {
            k_ave = (left_line[i] + left_line[i+1] + left_line[i+2] + left_line[i+3] + left_line[i+4] + right_line[i+1] + right_line[i] + right_line[i+2] + right_line[i+3] + right_line[i+4])/10;
            //取平均值，增强可靠性
            g = 1;//只找一次，节省时间
        }
        k_min = k_ave;
        if (k_ave < 94)
        {
            /*printf("ki");*/
            if (k <= k_ave)
            {
                k_min = k;
            }
            else if (k > k_ave)
            {
                k = k_min;
            }
        }
        else if (k_ave > 94)
        {
            /*printf("km");*/
            if (k >= k_ave)
            {
                k_min = k;
            }
            else if (k < k_ave)
            {
                k = k_min;
            }
        }
        else if (k_ave == 0)
        {
            i = 5;//下方全为白色，跳出此函数
            /*printf("kk");*/
        }
        /*IMG[i][left_line[i]] = blue;
        IMG[i][right_line[i]] = red;*/
    }
}
//////////////////////////////////////////
//功能：十字边界处理1
//输入：
//输出：
//备注：
///////////////////////////////////////////
void get_cross_line1(void)
{
    for (int i = 90; i > 10; i--)
    {
        if ((((int)left_line[i] - (int)left_line[i - 5]) >=5  )||(((int)right_line[i - 5] - (int)right_line[i]) >= 5)||((wide_line[i + 2] - wide_line[i]) > 20))
        {
            pio_x[0] = left_line[i];
            pio_y[0] = i;
            pio_x[1] = right_line[i];
            pio_y[1] = i;
            i = 10;
        }
    }
    if ((pio_x[0] != 0) && (pio_y[0] != 0) && (pio_y[1] != 0) && (pio_y[1] != 0))
    {
        for (int j = 20; j < pio_y[0]; j++)
        {
            left_line[j] = (uint8_t)((float)left_line[pio_y[0]+10] + (float)(pio_y[0] + 10-j) / (float)(10) * (float)(pio_x[0] - left_line[pio_y[0] + 10]));
            right_line[j] = (uint8_t)((float)right_line[pio_y[0] + 10] - (float)(pio_y[0] + 10 - j) / (float)(10) * (float)(right_line[pio_y[0] + 10] - pio_x[1]));
            if ((left_line[j] < 3) || (right_line[j] > 185))
            {
                j = 188;
            }//防止数组横坐标越界
            IMG[j][left_line[j]] = green;
            IMG[j][right_line[j]] = purple;
        }
    }
}
////////////////////////////////////////////
//功能：十字边界处理2
//输入：
//输出：
//备注：
///////////////////////////////////////////
void get_cross_line2(void)
{
    for (int i = 20; i <100; i++)
    {
        if ((((int)left_line[i] - (int)left_line[i+3]) > 8) || (((int)right_line[i+3] - (int)right_line[i]) > 8)||((wide_line[i + 2] - wide_line[i]) > 20 && (wide_line[i] > 30)))
        {
            pio_x[2] = left_line[i];
            pio_y[2] = i;
            pio_x[3] = right_line[i];
            pio_y[3] = i;
            i = 100;
        }
    }
    if ((pio_x[2] != 0) && (pio_y[2] != 0) && (pio_y[3] != 0) && (pio_y[3] != 0))
    {
        for (int j = pio_y[2]; j < 100; j++)
        {
            left_line[j] = (uint8_t)((float)left_line[pio_y[2] - 5] - (float)(j - pio_y[2] + 5) / (float)(5) * (float)(left_line[pio_y[2] - 5] - pio_x[2]));
            right_line[j] = (uint8_t)((float)right_line[pio_y[3] - 5] + (float)(j - pio_y[3] + 5) / (float)(5) * (float)(pio_x[3] - right_line[pio_y[3] - 5]));
            if ((left_line[j] < 3) || (right_line[j] > 185))
            {
                j = 188;
            }//防止数组横坐标越界
            if ((left_line[j] > 120) || (right_line[j] <70))
            {
                j = 100;
            }
            IMG[j][left_line[j]] = green;
            IMG[j][right_line[j]] = purple;
        }
    }
}

////////////////////////////////////////////
//功能：中线点修正
//输入：
//输出：
//备注：
///////////////////////////////////////////
void midline_check(void)
{
    if (mid_line[any + 1] - mid_line[any] > 1)
    {
        mid_line[any] = mid_line[any + 1] - 1;
    }
    else if (mid_line[any] - mid_line[any + 1] > 1)
    {
        mid_line[any] = mid_line[any + 1] + 1;
    }
}
////////////////////////////////////////////
//功能：初始化数组
//输入：
//输出：
//备注：
///////////////////////////////////////////
void init(void)
{
    line_type = 0;
    for (int i = 0; i < 120; i++)
    {
        left_line[i] = 0;
        right_line[i] = 0;
        mid_line[i] = 0;
        wide_line[i] = 0;
        leftline_change[i] = 0;
    }
    for (int j = 0; j < 4; j++)
    {
        pio_x[j] = 0;
        pio_y[j] = 0;
    }
}
////////////////////////////////////////////
//功能：斑马线检测
//输入：
//输出：
//备注：
///////////////////////////////////////////
void zebra_check(void)
{
    uint8_t istart = 100;
    uint8_t iend = 10;
    uint8_t mark_num, j_min, j_max;
    road* my_line = NULL;
    int range_num = 0;
    for (int i = istart - 1; i >= iend; i--)
    {
        my_line = &my_road[i];
        range_num = my_line->white_num;
        if (left_line[i + 1] != MISS && range_num != 0)
        {
            mark_num = 0;
            j_min = range_num;
            j_max = 1;
            for (int j = 1; j <= range_num; j++)//判断是否连通
            {
                if (my_line->connected[j].left <= right_line[i + 1] && my_line->connected[j].right >= left_line[i + 1])
                {
                    if (j < j_min) j_min = j;
                    if (j > j_max) j_max = j;
                    mark_num++;//联通白条数
                }
            }
            if (mark_num == 0)
            {
                left_line[i] = MISS; right_line[i] = MISS;
            }
            if (mark_num == 1)
            {
                left_line[i] = my_line->connected[j_min].left; right_line[i] = my_line->connected[j_min].right;
            }
            else
            {
                if (mark_num >= 5 )
                {
                    line_type = 4;
                    /*printf("\nsssss\n");
                    printf("\nss%d\n", mark_num);*/
                }
                left_line[i] = my_line->connected[j_min].left;
                right_line[i] = my_line->connected[j_max].right;
            }
        }
        if (left_line[i] < 0) left_line[i] = MISS;
        if (right_line[i] > 187) right_line[i] = MISS;
    }
}
////////////////////////////////////////////
//功能：斑马线停车
//输入：
//输出：
//备注：
///////////////////////////////////////////
void zebra_stop(void)
{
    int point_num=0;
    zebra_flag2 = 0;
    for (int j = 60; j < 130; j++)
    {
        if ((IMG[70][j] == 0) && (IMG[70][j + 1] == 1))
        {
            point_num++;
        }
    }
    if (point_num >= 3)
    {
        zebra_flag2 = 1;
    }
}
////////////////////////////////////////////
//功能：图像处理主程序
//输入：
//输出：
//备注：
///////////////////////////////////////////
void image_main()
{
    init();
    //head_clear();
    search_white_range();
    find_all_connect();
    find_road();
    ordinary_two_line();
    type_line();
    /*zebra_check();*/
    if (line_type == 1)
    {
        printf("aaa\n");
        get_cross_line1();
        zebra_flag = 1;
    }
    else if (line_type == 2)
    {
        printf("bbb\n");
        cross_two_line2();
        get_cross_line2();
        zebra_flag = 1;//经过十字，开启过斑马线停车
    }
    get_mid_line();
    if ((line_type == 4) && (zebra_flag == 1))
    {
        //写一个定时中断函数
        zebra_flag = 0;//只开启一次中断
    }
    printf("%d %d %d %d\n", pio_x[0], pio_y[0], pio_x[1], pio_y[1]);
    printf("%d %d %d %d\n\n", pio_x[2], pio_y[2], pio_x[3], pio_y[3]);//找BUG用
    for (int i = NEAR_LINE; i >= FAR_LINE; i--)
        if (mid_line[i] != MISS)
            IMG[i][mid_line[i]] = gray;
//    for (int j = 40; j < 45; j++)
//    {
//        IMG[j][94] = red;
//    }
//    for (int j = 60; j < 71; j++)
//    {
//        IMG[j][94] = red;
//    }

    //for (int i = 0; i < 120; i++)
    //  for (int j = 1; j <= my_road[i].white_num; j++)
    //      for (int k = my_road[i].connected[j].left; k <= my_road[i].connected[j].right; k++)
    //          IMG[i][k] = gray;
}


