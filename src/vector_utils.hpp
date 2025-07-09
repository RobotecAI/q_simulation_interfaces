#pragma once

#include <QString>
#include <tf2/LinearMath/Vector3.h>
#include <QStringList>
#include <QDebug>

inline QString VectorToQstring (const tf2::Vector3& v )
{
    QString qString = QString::asprintf("%f %f %f", v.x(), v.y(), v.z());
    return qString;
}

inline tf2::Vector3 QStringToVector (const QString& line )
{
    tf2::Vector3 v;
    QStringList list = line.split(" ");
    if (list.size() == 3) {
        v.setX(list[0].toDouble());
        v.setY(list[1].toDouble());
        v.setZ(list[2].toDouble());
    } else {
        qWarning() << "Invalid input format. Expected 3 values.";
        v = tf2::Vector3(0, 0, 0);
    }
    return v;
}