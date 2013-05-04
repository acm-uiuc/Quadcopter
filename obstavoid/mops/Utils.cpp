#include "Utils.h"

bool isRectWithinImage(const Mat& image, const Rect rect)
{
    //if (rect.x == 0 && rect.y == 303)
    //exit(EXIT_FAILURE);
    //cout << image.rows << " " << image.cols << endl;
    //cout << rect.y << " " << rect.x << endl;
    if (rect.x < 0 || rect.y < 0)
	return false;
    if ((rect.x + rect.width) >= image.cols)
	return false;
    if ((rect.y + rect.height) >= image.rows)
	return false;
    return true;
}
