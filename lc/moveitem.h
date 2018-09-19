#ifndef MOVEITEM_H
#define MOVEITEM_H

#include <QObject>
#include <QGraphicsItem>
#include <QPainter>
#include <QGraphicsSceneMouseEvent>
#include <QDebug>
#include <QCursor>
#include <QLabel>
#include <QLineEdit>
#include <QLayout>

class MoveItem : public QObject, public QGraphicsItem
{
    Q_OBJECT

public:
    explicit MoveItem(QObject *parent = 0, QLineEdit *x_pos = 0, QLineEdit *y_pos = 0);
    ~MoveItem();

signals:

private:
    QRectF boundingRect() const;
    void paint(QPainter *painter, const QStyleOptionGraphicsItem *option, QWidget *widget);
    void mouseMoveEvent(QGraphicsSceneMouseEvent *event);
    void mousePressEvent(QGraphicsSceneMouseEvent *event);
    void mouseReleaseEvent(QGraphicsSceneMouseEvent *event);
    QLineEdit *xp_pntr;
    QLineEdit *yp_pntr;


public slots:


};


#endif // MOVEITEM_H
