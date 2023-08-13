#include "common.h"

//自定义函数

bool MyFunctions::condition = true;

void MyFunctions::stop(int sign)
{
    MyFunctions::condition = false;
    std::cout << "\nUser's Interrupt" << std::endl;
    //exit(0);
}

const bool MyFunctions::ok()
{
    //std::cout << "MyFunctions::ok()" << std::endl;
    return MyFunctions::condition;
}
