#ifndef MYLABLE_H
#define MYLABLE_H

#include <QLabel>
#include <QMouseEvent>

extern int x_in_Label, y_in_Label, x_in_Ui, y_in_Ui, x_in_Img, y_in_Img;

class myLable :public QLabel
{
    Q_OBJECT

public:
    myLable(QWidget *parent = 0);
    ~myLable();

public:
    void mousePressEvent(QMouseEvent *e);    //按下

};

#endif // MYLABLE_H
