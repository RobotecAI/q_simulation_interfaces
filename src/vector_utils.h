/* Copyright 2025, Robotec.ai sp. z o.o.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#pragma once

#include <QDebug>
#include <QString>
#include <QStringList>
#include <tf2/LinearMath/Vector3.h>

inline QString VectorToQstring(const tf2::Vector3& v)
{
    QString qString = QString::asprintf("%f %f %f", v.x(), v.y(), v.z());
    return qString;
}

inline tf2::Vector3 QStringToVector(const QString& line)
{
    tf2::Vector3 v;
    QStringList list = line.split(" ");
    if (list.size() == 3)
    {
        v.setX(list[0].toDouble());
        v.setY(list[1].toDouble());
        v.setZ(list[2].toDouble());
    }
    else
    {
        qWarning() << "Invalid input format. Expected 3 values.";
        v = tf2::Vector3(0, 0, 0);
    }
    return v;
}
