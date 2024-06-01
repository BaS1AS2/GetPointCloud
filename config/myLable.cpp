#include "myLable.h"
#include <iostream>

int x_in_Label, y_in_Label, x_in_Ui, y_in_Ui, x_in_Img, y_in_Img;

myLable::myLable(QWidget* parent) :QLabel(parent)
{

}

myLable::~myLable()
{

}

void myLable::mousePressEvent(QMouseEvent *e)
{
    x_in_Label = e->x();
    y_in_Label = e->y();
    x_in_Ui = geometry().x() + x_in_Label;
    y_in_Ui = geometry().y() + y_in_Label;
    //这里需要知道输入图像的大小，然后根据缩放关系求出相应位置
    //    x_in_Img = x_in_Label* size.imageSizeX/width();
    //    y_in_Img = y_in_Label * size.imageSizeY/height();
    x_in_Img = x_in_Label * 800 / width();
    y_in_Img = y_in_Label * 600 / height();
//    std::cout << "mouse_in_label: "<< x_in_Label <<","<< y_in_Label
//                      <<  "    mouse_in_Ui: " << x_in_Ui << "," << y_in_Ui
//                      << "    mouse_in_Img: " <<x_in_Img <<"," << y_in_Img<< std::endl;
}
