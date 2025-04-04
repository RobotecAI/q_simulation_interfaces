/********************************************************************************
** Form generated from reading UI file 'my_widget.ui'
**
** Created by: Qt User Interface Compiler version 5.15.3
**
** WARNING! All changes made in this file will be lost when recompiling UI file!
********************************************************************************/

#ifndef UI_MY_WIDGET_H
#define UI_MY_WIDGET_H

#include <QtCore/QVariant>
#include <QtWidgets/QApplication>
#include <QtWidgets/QComboBox>
#include <QtWidgets/QDoubleSpinBox>
#include <QtWidgets/QGridLayout>
#include <QtWidgets/QHBoxLayout>
#include <QtWidgets/QLabel>
#include <QtWidgets/QLineEdit>
#include <QtWidgets/QPushButton>
#include <QtWidgets/QSpacerItem>
#include <QtWidgets/QVBoxLayout>
#include <QtWidgets/QWidget>

QT_BEGIN_NAMESPACE

class Ui_MyWidgetUI
{
public:
    QGridLayout *gridLayout;
    QVBoxLayout *verticalLayout;
    QLabel *label;
    QPushButton *getAllEntitiesButton;
    QComboBox *ComboEntities;
    QPushButton *despawnButton;
    QVBoxLayout *verticalLayout_3;
    QPushButton *getEntityStateButton;
    QGridLayout *gridLayout_2;
    QLabel *label_6;
    QDoubleSpinBox *StatePosX;
    QLabel *label_7;
    QDoubleSpinBox *StatePosY;
    QDoubleSpinBox *StatePosZ;
    QLabel *label_8;
    QPushButton *setEntityStateButton;
    QVBoxLayout *verticalLayout_2;
    QLabel *label_2;
    QHBoxLayout *horizontalLayout_2;
    QPushButton *PushButtonRefresh;
    QComboBox *ComboSpawables;
    QHBoxLayout *horizontalLayout_5;
    QLabel *label_3;
    QDoubleSpinBox *doubleSpinBoxX;
    QLabel *label_4;
    QDoubleSpinBox *doubleSpinBoxY;
    QLabel *label_5;
    QDoubleSpinBox *doubleSpinBoxZ;
    QHBoxLayout *horizontalLayout;
    QLabel *Name;
    QLineEdit *lineEditName;
    QPushButton *SpawnButton;
    QSpacerItem *verticalSpacer;

    void setupUi(QWidget *MyWidgetUI)
    {
        if (MyWidgetUI->objectName().isEmpty())
            MyWidgetUI->setObjectName(QString::fromUtf8("MyWidgetUI"));
        MyWidgetUI->resize(672, 724);
        gridLayout = new QGridLayout(MyWidgetUI);
        gridLayout->setObjectName(QString::fromUtf8("gridLayout"));
        verticalLayout = new QVBoxLayout();
        verticalLayout->setObjectName(QString::fromUtf8("verticalLayout"));
        label = new QLabel(MyWidgetUI);
        label->setObjectName(QString::fromUtf8("label"));

        verticalLayout->addWidget(label);

        getAllEntitiesButton = new QPushButton(MyWidgetUI);
        getAllEntitiesButton->setObjectName(QString::fromUtf8("getAllEntitiesButton"));

        verticalLayout->addWidget(getAllEntitiesButton);

        ComboEntities = new QComboBox(MyWidgetUI);
        ComboEntities->setObjectName(QString::fromUtf8("ComboEntities"));

        verticalLayout->addWidget(ComboEntities);

        despawnButton = new QPushButton(MyWidgetUI);
        despawnButton->setObjectName(QString::fromUtf8("despawnButton"));

        verticalLayout->addWidget(despawnButton);

        verticalLayout_3 = new QVBoxLayout();
        verticalLayout_3->setObjectName(QString::fromUtf8("verticalLayout_3"));
        getEntityStateButton = new QPushButton(MyWidgetUI);
        getEntityStateButton->setObjectName(QString::fromUtf8("getEntityStateButton"));

        verticalLayout_3->addWidget(getEntityStateButton);

        gridLayout_2 = new QGridLayout();
        gridLayout_2->setObjectName(QString::fromUtf8("gridLayout_2"));
        label_6 = new QLabel(MyWidgetUI);
        label_6->setObjectName(QString::fromUtf8("label_6"));

        gridLayout_2->addWidget(label_6, 0, 1, 1, 1);

        StatePosX = new QDoubleSpinBox(MyWidgetUI);
        StatePosX->setObjectName(QString::fromUtf8("StatePosX"));
        StatePosX->setMinimum(-999999.000000000000000);
        StatePosX->setMaximum(999999999999999.000000000000000);

        gridLayout_2->addWidget(StatePosX, 0, 2, 1, 1);

        label_7 = new QLabel(MyWidgetUI);
        label_7->setObjectName(QString::fromUtf8("label_7"));

        gridLayout_2->addWidget(label_7, 1, 1, 1, 1);

        StatePosY = new QDoubleSpinBox(MyWidgetUI);
        StatePosY->setObjectName(QString::fromUtf8("StatePosY"));
        StatePosY->setMinimum(-999999.000000000000000);
        StatePosY->setMaximum(999999999999999.000000000000000);

        gridLayout_2->addWidget(StatePosY, 1, 2, 1, 1);

        StatePosZ = new QDoubleSpinBox(MyWidgetUI);
        StatePosZ->setObjectName(QString::fromUtf8("StatePosZ"));
        StatePosZ->setMinimum(-999999.000000000000000);
        StatePosZ->setMaximum(999999999999999.000000000000000);

        gridLayout_2->addWidget(StatePosZ, 2, 2, 1, 1);

        label_8 = new QLabel(MyWidgetUI);
        label_8->setObjectName(QString::fromUtf8("label_8"));

        gridLayout_2->addWidget(label_8, 2, 1, 1, 1);


        verticalLayout_3->addLayout(gridLayout_2);

        setEntityStateButton = new QPushButton(MyWidgetUI);
        setEntityStateButton->setObjectName(QString::fromUtf8("setEntityStateButton"));

        verticalLayout_3->addWidget(setEntityStateButton);


        verticalLayout->addLayout(verticalLayout_3);


        gridLayout->addLayout(verticalLayout, 0, 0, 1, 1);

        verticalLayout_2 = new QVBoxLayout();
        verticalLayout_2->setObjectName(QString::fromUtf8("verticalLayout_2"));
        label_2 = new QLabel(MyWidgetUI);
        label_2->setObjectName(QString::fromUtf8("label_2"));

        verticalLayout_2->addWidget(label_2);

        horizontalLayout_2 = new QHBoxLayout();
        horizontalLayout_2->setObjectName(QString::fromUtf8("horizontalLayout_2"));
        PushButtonRefresh = new QPushButton(MyWidgetUI);
        PushButtonRefresh->setObjectName(QString::fromUtf8("PushButtonRefresh"));

        horizontalLayout_2->addWidget(PushButtonRefresh);

        ComboSpawables = new QComboBox(MyWidgetUI);
        ComboSpawables->setObjectName(QString::fromUtf8("ComboSpawables"));

        horizontalLayout_2->addWidget(ComboSpawables);


        verticalLayout_2->addLayout(horizontalLayout_2);

        horizontalLayout_5 = new QHBoxLayout();
        horizontalLayout_5->setObjectName(QString::fromUtf8("horizontalLayout_5"));
        label_3 = new QLabel(MyWidgetUI);
        label_3->setObjectName(QString::fromUtf8("label_3"));

        horizontalLayout_5->addWidget(label_3);

        doubleSpinBoxX = new QDoubleSpinBox(MyWidgetUI);
        doubleSpinBoxX->setObjectName(QString::fromUtf8("doubleSpinBoxX"));
        doubleSpinBoxX->setMinimum(-999999.000000000000000);
        doubleSpinBoxX->setMaximum(999999999999999.000000000000000);

        horizontalLayout_5->addWidget(doubleSpinBoxX);

        label_4 = new QLabel(MyWidgetUI);
        label_4->setObjectName(QString::fromUtf8("label_4"));

        horizontalLayout_5->addWidget(label_4);

        doubleSpinBoxY = new QDoubleSpinBox(MyWidgetUI);
        doubleSpinBoxY->setObjectName(QString::fromUtf8("doubleSpinBoxY"));
        doubleSpinBoxY->setMinimum(-999999.000000000000000);
        doubleSpinBoxY->setMaximum(999999999999999.000000000000000);

        horizontalLayout_5->addWidget(doubleSpinBoxY);

        label_5 = new QLabel(MyWidgetUI);
        label_5->setObjectName(QString::fromUtf8("label_5"));

        horizontalLayout_5->addWidget(label_5);

        doubleSpinBoxZ = new QDoubleSpinBox(MyWidgetUI);
        doubleSpinBoxZ->setObjectName(QString::fromUtf8("doubleSpinBoxZ"));
        doubleSpinBoxZ->setMinimum(-999999.000000000000000);
        doubleSpinBoxZ->setMaximum(999999999999999.000000000000000);

        horizontalLayout_5->addWidget(doubleSpinBoxZ);


        verticalLayout_2->addLayout(horizontalLayout_5);

        horizontalLayout = new QHBoxLayout();
        horizontalLayout->setObjectName(QString::fromUtf8("horizontalLayout"));
        Name = new QLabel(MyWidgetUI);
        Name->setObjectName(QString::fromUtf8("Name"));

        horizontalLayout->addWidget(Name);

        lineEditName = new QLineEdit(MyWidgetUI);
        lineEditName->setObjectName(QString::fromUtf8("lineEditName"));

        horizontalLayout->addWidget(lineEditName);


        verticalLayout_2->addLayout(horizontalLayout);

        SpawnButton = new QPushButton(MyWidgetUI);
        SpawnButton->setObjectName(QString::fromUtf8("SpawnButton"));

        verticalLayout_2->addWidget(SpawnButton);

        verticalSpacer = new QSpacerItem(20, 40, QSizePolicy::Minimum, QSizePolicy::Expanding);

        verticalLayout_2->addItem(verticalSpacer);


        gridLayout->addLayout(verticalLayout_2, 1, 0, 1, 1);


        retranslateUi(MyWidgetUI);

        QMetaObject::connectSlotsByName(MyWidgetUI);
    } // setupUi

    void retranslateUi(QWidget *MyWidgetUI)
    {
        label->setText(QCoreApplication::translate("MyWidgetUI", "Entities State", nullptr));
        getAllEntitiesButton->setText(QCoreApplication::translate("MyWidgetUI", "GetAllEntities", nullptr));
        despawnButton->setText(QCoreApplication::translate("MyWidgetUI", "Despawn", nullptr));
        getEntityStateButton->setText(QCoreApplication::translate("MyWidgetUI", "GetEntityState", nullptr));
        label_6->setText(QCoreApplication::translate("MyWidgetUI", "PosX:", nullptr));
        label_7->setText(QCoreApplication::translate("MyWidgetUI", "PosY:", nullptr));
        label_8->setText(QCoreApplication::translate("MyWidgetUI", "PosZ:", nullptr));
        setEntityStateButton->setText(QCoreApplication::translate("MyWidgetUI", "SetEntityState", nullptr));
        label_2->setText(QCoreApplication::translate("MyWidgetUI", "Spawning", nullptr));
        PushButtonRefresh->setText(QCoreApplication::translate("MyWidgetUI", "Refresh", nullptr));
        label_3->setText(QCoreApplication::translate("MyWidgetUI", "X:", nullptr));
        label_4->setText(QCoreApplication::translate("MyWidgetUI", "Y:", nullptr));
        label_5->setText(QCoreApplication::translate("MyWidgetUI", "Z:", nullptr));
        Name->setText(QCoreApplication::translate("MyWidgetUI", "Name", nullptr));
        SpawnButton->setText(QCoreApplication::translate("MyWidgetUI", "Spawn!", nullptr));
        (void)MyWidgetUI;
    } // retranslateUi

};

namespace Ui {
    class MyWidgetUI: public Ui_MyWidgetUI {};
} // namespace Ui

QT_END_NAMESPACE

#endif // UI_MY_WIDGET_H
