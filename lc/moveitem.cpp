#include "moveitem.h"

MoveItem::MoveItem(QObject *parent, QLineEdit *x_pos, QLineEdit *y_pos) :
    QObject(parent), QGraphicsItem()
{

    this->setCursor(QCursor(Qt::OpenHandCursor));

    //le_pointer = line_edit;
    //le_pointer->setText("hello");

    xp_pntr = x_pos;
    yp_pntr = y_pos;

}

MoveItem::~MoveItem()
{


}

QRectF MoveItem::boundingRect() const
{

    return QRectF(-7,-7,14,14);

}

void MoveItem::paint(QPainter *painter, const QStyleOptionGraphicsItem *option, QWidget *widget){

    painter->setPen(Qt::black);
    painter->setBrush(Qt::blue);
    painter->drawEllipse(-7,-7,14,14);
    Q_UNUSED(option);
    Q_UNUSED(widget);
}

void MoveItem::mouseMoveEvent(QGraphicsSceneMouseEvent *event){

    this->setPos(mapToScene(event->pos()));
    //le_pointer->setText("("+QString::number(this->x()) +" ; "+ QString::number(this->y())+")");
    xp_pntr->setText(QString::number(this->x()/10));
    yp_pntr->setText(QString::number(-this->y()/10));


}

void MoveItem::mousePressEvent(QGraphicsSceneMouseEvent *event){

    this->setCursor(QCursor(Qt::ClosedHandCursor));

    //le_pointer->setText(QString::number(this->x()));
    xp_pntr->setText(QString::number(this->x()/10));
    yp_pntr->setText(QString::number(-this->y()/10));

    Q_UNUSED(event);


}

void MoveItem::mouseReleaseEvent(QGraphicsSceneMouseEvent *event){

    this->setCursor(QCursor(Qt::OpenHandCursor));

    //le_pointer->setText("("+QString::number(this->x()) +" ; "+ QString::number(this->y())+")");

    Q_UNUSED(event);

}
