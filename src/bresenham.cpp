/**
 * 
 * 1. 计算出斜率与截距
 * ２．根据斜率的正负，选择合适的增量
 * ３．使用增量计算出下一个像素点的位置
 * ４．重复步骤３，直到绘制完整条直线
 * 
*/
void bresenham(int x1, int y1, int x2, int y2)
{
    int dx, dy, p, x, y;

    dx = x2 - x1;
    dy = y2 - y1;

    x = x1;
    y = y1;

    p = 2 * dy - dx;

    while (x < x2)
    {
        if (p >= 0)
        {
            cout << x << ' ' << y << endl;
            y = y + 1;
            p = p + 2 * dy - 2 * dx;
        }
        else
        {
            cout << x << ' ' << y << endl;
            p = p + 2 * dy;
        }
        x = x + 1;
    }
}