/********************************************************************************
** Form generated from reading UI file 'dummy_window.ui'
**
** Created by: Qt User Interface Compiler version 4.8.7
**
** WARNING! All changes made in this file will be lost when recompiling UI file!
********************************************************************************/

#ifndef DOTNECT_WINDOW_H
#define DOTNECT_WINDOW_H

#include <QtCore/QVariant>
#include <QtGui/QAction>
#include <QtGui/QApplication>
#include <QtGui/QButtonGroup>
#include <QtGui/QCheckBox>
#include <QtGui/QGridLayout>
#include <QtGui/QHBoxLayout>
#include <QtGui/QHeaderView>
#include <QtGui/QLabel>
#include <QtGui/QLineEdit>
#include <QtGui/QPushButton>
#include <QtGui/QRadioButton>
#include <QtGui/QSlider>
#include <QtGui/QSpacerItem>
#include <QtGui/QVBoxLayout>
#include <QtGui/QWidget>

QT_BEGIN_NAMESPACE

class Ui_dummy_form
{
public:
  QGridLayout *gridLayout;
  QVBoxLayout *verticalLayout_6;
  QLabel *label_2;
  QHBoxLayout *horizontalLayout_2;
  QLabel *label;
  QLineEdit *lineEdit;
  QCheckBox *checkBox;
  QCheckBox *checkBox_4;
  QHBoxLayout *horizontalLayout_3;
  QLabel *label_4;
  QRadioButton *radioButton;
  QRadioButton *radioButton_2;
  QRadioButton *radioButton_3;
  QSpacerItem *horizontalSpacer;
  QHBoxLayout *horizontalLayout_4;
  QLabel *label_7;
  QSlider *horizontalSlider;
  QLabel *label_8;
  QSpacerItem *verticalSpacer_3;
  QLabel *label_3;
  QCheckBox *checkBox_2;
  QSpacerItem *verticalSpacer;
  QLabel *label_5;
  QHBoxLayout *horizontalLayout_5;
  QCheckBox *checkBox_3;
  QCheckBox *checkBox_5;
  QSpacerItem *horizontalSpacer_2;
  QHBoxLayout *horizontalLayout_6;
  QLabel *label_9;
  QSlider *horizontalSlider_2;
  QLabel *label_6;
  QHBoxLayout *horizontalLayout_9;
  QLabel *label_12;
  QSlider *horizontalSlider_3;
  QLabel *label_15;
  QHBoxLayout *horizontalLayout_10;
  QLabel *label_13;
  QSlider *horizontalSlider_4;
  QLabel *label_16;
  QHBoxLayout *horizontalLayout_11;
  QLabel *label_14;
  QSlider *horizontalSlider_5;
  QLabel *label_17;
  QHBoxLayout *horizontalLayout_12;
  QLabel *label_10;
  QSlider *horizontalSlider_6;
  QLabel *label_18;
  QHBoxLayout *horizontalLayout_13;
  QLabel *label_11;
  QSlider *horizontalSlider_7;
  QLabel *label_19;
  QSpacerItem *verticalSpacer_4;
  QHBoxLayout *horizontalLayout_8;
  QLabel *label_20;
  QRadioButton *radioButton_4;
  QRadioButton *radioButton_5;
  QSpacerItem *horizontalSpacer_3;
  QHBoxLayout *horizontalLayout_14;
  QLabel *label_21;
  QLineEdit *lineEdit_2;
  QLineEdit *lineEdit_3;
  QPushButton *pushButton;
  QSpacerItem *verticalSpacer_2;

  void setupUi(QWidget *dummy_form)
  {
    if (dummy_form->objectName().isEmpty())
      dummy_form->setObjectName(QString::fromUtf8("dummy_form"));
    dummy_form->resize(422, 624);
    QSizePolicy sizePolicy(QSizePolicy::Preferred, QSizePolicy::Preferred);
    sizePolicy.setHorizontalStretch(0);
    sizePolicy.setVerticalStretch(0);
    sizePolicy.setHeightForWidth(dummy_form->sizePolicy().hasHeightForWidth());
    dummy_form->setSizePolicy(sizePolicy);
    gridLayout = new QGridLayout(dummy_form);
    gridLayout->setSpacing(6);
    gridLayout->setObjectName(QString::fromUtf8("gridLayout"));
    gridLayout->setContentsMargins(9, 9, 9, 9);
    verticalLayout_6 = new QVBoxLayout();
    verticalLayout_6->setObjectName(QString::fromUtf8("verticalLayout_6"));
    label_2 = new QLabel(dummy_form);
    label_2->setObjectName(QString::fromUtf8("label_2"));
    QFont font;
    font.setBold(true);
    font.setWeight(75);
    label_2->setFont(font);

    verticalLayout_6->addWidget(label_2);

    horizontalLayout_2 = new QHBoxLayout();
    horizontalLayout_2->setObjectName(QString::fromUtf8("horizontalLayout_2"));
    label = new QLabel(dummy_form);
    label->setObjectName(QString::fromUtf8("label"));

    horizontalLayout_2->addWidget(label);

    lineEdit = new QLineEdit(dummy_form);
    lineEdit->setObjectName(QString::fromUtf8("lineEdit"));

    horizontalLayout_2->addWidget(lineEdit);

    verticalLayout_6->addLayout(horizontalLayout_2);

    checkBox = new QCheckBox(dummy_form);
    checkBox->setObjectName(QString::fromUtf8("checkBox"));

    verticalLayout_6->addWidget(checkBox);

    checkBox_4 = new QCheckBox(dummy_form);
    checkBox_4->setObjectName(QString::fromUtf8("checkBox_4"));

    verticalLayout_6->addWidget(checkBox_4);

    horizontalLayout_3 = new QHBoxLayout();
    horizontalLayout_3->setObjectName(QString::fromUtf8("horizontalLayout_3"));
    label_4 = new QLabel(dummy_form);
    label_4->setObjectName(QString::fromUtf8("label_4"));

    horizontalLayout_3->addWidget(label_4);

    radioButton = new QRadioButton(dummy_form);
    radioButton->setObjectName(QString::fromUtf8("radioButton"));

    horizontalLayout_3->addWidget(radioButton);

    radioButton_2 = new QRadioButton(dummy_form);
    radioButton_2->setObjectName(QString::fromUtf8("radioButton_2"));

    horizontalLayout_3->addWidget(radioButton_2);

    radioButton_3 = new QRadioButton(dummy_form);
    radioButton_3->setObjectName(QString::fromUtf8("radioButton_3"));

    horizontalLayout_3->addWidget(radioButton_3);

    horizontalSpacer = new QSpacerItem(40, 20, QSizePolicy::Expanding, QSizePolicy::Minimum);

    horizontalLayout_3->addItem(horizontalSpacer);

    verticalLayout_6->addLayout(horizontalLayout_3);

    horizontalLayout_4 = new QHBoxLayout();
    horizontalLayout_4->setObjectName(QString::fromUtf8("horizontalLayout_4"));
    label_7 = new QLabel(dummy_form);
    label_7->setObjectName(QString::fromUtf8("label_7"));

    horizontalLayout_4->addWidget(label_7);

    horizontalSlider = new QSlider(dummy_form);
    horizontalSlider->setObjectName(QString::fromUtf8("horizontalSlider"));
    horizontalSlider->setOrientation(Qt::Horizontal);

    horizontalLayout_4->addWidget(horizontalSlider);

    label_8 = new QLabel(dummy_form);
    label_8->setObjectName(QString::fromUtf8("label_8"));

    horizontalLayout_4->addWidget(label_8);

    verticalLayout_6->addLayout(horizontalLayout_4);

    verticalSpacer_3 = new QSpacerItem(20, 40, QSizePolicy::Minimum, QSizePolicy::Expanding);

    verticalLayout_6->addItem(verticalSpacer_3);

    label_3 = new QLabel(dummy_form);
    label_3->setObjectName(QString::fromUtf8("label_3"));
    label_3->setFont(font);

    verticalLayout_6->addWidget(label_3);

    checkBox_2 = new QCheckBox(dummy_form);
    checkBox_2->setObjectName(QString::fromUtf8("checkBox_2"));

    verticalLayout_6->addWidget(checkBox_2);

    verticalSpacer = new QSpacerItem(20, 40, QSizePolicy::Minimum, QSizePolicy::Expanding);

    verticalLayout_6->addItem(verticalSpacer);

    label_5 = new QLabel(dummy_form);
    label_5->setObjectName(QString::fromUtf8("label_5"));
    label_5->setFont(font);

    verticalLayout_6->addWidget(label_5);

    horizontalLayout_5 = new QHBoxLayout();
    horizontalLayout_5->setObjectName(QString::fromUtf8("horizontalLayout_5"));
    checkBox_3 = new QCheckBox(dummy_form);
    checkBox_3->setObjectName(QString::fromUtf8("checkBox_3"));

    horizontalLayout_5->addWidget(checkBox_3);

    checkBox_5 = new QCheckBox(dummy_form);
    checkBox_5->setObjectName(QString::fromUtf8("checkBox_5"));

    horizontalLayout_5->addWidget(checkBox_5);

    horizontalSpacer_2 = new QSpacerItem(40, 20, QSizePolicy::Expanding, QSizePolicy::Minimum);

    horizontalLayout_5->addItem(horizontalSpacer_2);

    verticalLayout_6->addLayout(horizontalLayout_5);

    horizontalLayout_6 = new QHBoxLayout();
    horizontalLayout_6->setObjectName(QString::fromUtf8("horizontalLayout_6"));
    label_9 = new QLabel(dummy_form);
    label_9->setObjectName(QString::fromUtf8("label_9"));

    horizontalLayout_6->addWidget(label_9);

    horizontalSlider_2 = new QSlider(dummy_form);
    horizontalSlider_2->setObjectName(QString::fromUtf8("horizontalSlider_2"));
    horizontalSlider_2->setOrientation(Qt::Horizontal);

    horizontalLayout_6->addWidget(horizontalSlider_2);

    label_6 = new QLabel(dummy_form);
    label_6->setObjectName(QString::fromUtf8("label_6"));

    horizontalLayout_6->addWidget(label_6);

    verticalLayout_6->addLayout(horizontalLayout_6);

    horizontalLayout_9 = new QHBoxLayout();
    horizontalLayout_9->setObjectName(QString::fromUtf8("horizontalLayout_9"));
    label_12 = new QLabel(dummy_form);
    label_12->setObjectName(QString::fromUtf8("label_12"));

    horizontalLayout_9->addWidget(label_12);

    horizontalSlider_3 = new QSlider(dummy_form);
    horizontalSlider_3->setObjectName(QString::fromUtf8("horizontalSlider_3"));
    horizontalSlider_3->setOrientation(Qt::Horizontal);

    horizontalLayout_9->addWidget(horizontalSlider_3);

    label_15 = new QLabel(dummy_form);
    label_15->setObjectName(QString::fromUtf8("label_15"));

    horizontalLayout_9->addWidget(label_15);

    verticalLayout_6->addLayout(horizontalLayout_9);

    horizontalLayout_10 = new QHBoxLayout();
    horizontalLayout_10->setObjectName(QString::fromUtf8("horizontalLayout_10"));
    label_13 = new QLabel(dummy_form);
    label_13->setObjectName(QString::fromUtf8("label_13"));

    horizontalLayout_10->addWidget(label_13);

    horizontalSlider_4 = new QSlider(dummy_form);
    horizontalSlider_4->setObjectName(QString::fromUtf8("horizontalSlider_4"));
    horizontalSlider_4->setOrientation(Qt::Horizontal);

    horizontalLayout_10->addWidget(horizontalSlider_4);

    label_16 = new QLabel(dummy_form);
    label_16->setObjectName(QString::fromUtf8("label_16"));

    horizontalLayout_10->addWidget(label_16);

    verticalLayout_6->addLayout(horizontalLayout_10);

    horizontalLayout_11 = new QHBoxLayout();
    horizontalLayout_11->setObjectName(QString::fromUtf8("horizontalLayout_11"));
    label_14 = new QLabel(dummy_form);
    label_14->setObjectName(QString::fromUtf8("label_14"));

    horizontalLayout_11->addWidget(label_14);

    horizontalSlider_5 = new QSlider(dummy_form);
    horizontalSlider_5->setObjectName(QString::fromUtf8("horizontalSlider_5"));
    horizontalSlider_5->setOrientation(Qt::Horizontal);

    horizontalLayout_11->addWidget(horizontalSlider_5);

    label_17 = new QLabel(dummy_form);
    label_17->setObjectName(QString::fromUtf8("label_17"));

    horizontalLayout_11->addWidget(label_17);

    verticalLayout_6->addLayout(horizontalLayout_11);

    horizontalLayout_12 = new QHBoxLayout();
    horizontalLayout_12->setObjectName(QString::fromUtf8("horizontalLayout_12"));
    label_10 = new QLabel(dummy_form);
    label_10->setObjectName(QString::fromUtf8("label_10"));

    horizontalLayout_12->addWidget(label_10);

    horizontalSlider_6 = new QSlider(dummy_form);
    horizontalSlider_6->setObjectName(QString::fromUtf8("horizontalSlider_6"));
    horizontalSlider_6->setOrientation(Qt::Horizontal);

    horizontalLayout_12->addWidget(horizontalSlider_6);

    label_18 = new QLabel(dummy_form);
    label_18->setObjectName(QString::fromUtf8("label_18"));

    horizontalLayout_12->addWidget(label_18);

    verticalLayout_6->addLayout(horizontalLayout_12);

    horizontalLayout_13 = new QHBoxLayout();
    horizontalLayout_13->setObjectName(QString::fromUtf8("horizontalLayout_13"));
    horizontalLayout_13->setSizeConstraint(QLayout::SetDefaultConstraint);
    label_11 = new QLabel(dummy_form);
    label_11->setObjectName(QString::fromUtf8("label_11"));

    horizontalLayout_13->addWidget(label_11);

    horizontalSlider_7 = new QSlider(dummy_form);
    horizontalSlider_7->setObjectName(QString::fromUtf8("horizontalSlider_7"));
    horizontalSlider_7->setOrientation(Qt::Horizontal);

    horizontalLayout_13->addWidget(horizontalSlider_7);

    label_19 = new QLabel(dummy_form);
    label_19->setObjectName(QString::fromUtf8("label_19"));

    horizontalLayout_13->addWidget(label_19);

    verticalLayout_6->addLayout(horizontalLayout_13);

    verticalSpacer_4 = new QSpacerItem(20, 10, QSizePolicy::Minimum, QSizePolicy::Fixed);

    verticalLayout_6->addItem(verticalSpacer_4);

    horizontalLayout_8 = new QHBoxLayout();
    horizontalLayout_8->setObjectName(QString::fromUtf8("horizontalLayout_8"));
    label_20 = new QLabel(dummy_form);
    label_20->setObjectName(QString::fromUtf8("label_20"));

    horizontalLayout_8->addWidget(label_20);

    radioButton_4 = new QRadioButton(dummy_form);
    radioButton_4->setObjectName(QString::fromUtf8("radioButton_4"));

    horizontalLayout_8->addWidget(radioButton_4);

    radioButton_5 = new QRadioButton(dummy_form);
    radioButton_5->setObjectName(QString::fromUtf8("radioButton_5"));

    horizontalLayout_8->addWidget(radioButton_5);

    horizontalSpacer_3 = new QSpacerItem(40, 20, QSizePolicy::Expanding, QSizePolicy::Minimum);

    horizontalLayout_8->addItem(horizontalSpacer_3);

    verticalLayout_6->addLayout(horizontalLayout_8);

    horizontalLayout_14 = new QHBoxLayout();
    horizontalLayout_14->setObjectName(QString::fromUtf8("horizontalLayout_14"));
    label_21 = new QLabel(dummy_form);
    label_21->setObjectName(QString::fromUtf8("label_21"));

    horizontalLayout_14->addWidget(label_21);

    lineEdit_2 = new QLineEdit(dummy_form);
    lineEdit_2->setObjectName(QString::fromUtf8("lineEdit_2"));

    horizontalLayout_14->addWidget(lineEdit_2);

    lineEdit_3 = new QLineEdit(dummy_form);
    lineEdit_3->setObjectName(QString::fromUtf8("lineEdit_3"));

    horizontalLayout_14->addWidget(lineEdit_3);

    pushButton = new QPushButton(dummy_form);
    pushButton->setObjectName(QString::fromUtf8("pushButton"));

    horizontalLayout_14->addWidget(pushButton);

    verticalLayout_6->addLayout(horizontalLayout_14);

    gridLayout->addLayout(verticalLayout_6, 0, 0, 1, 1);

    verticalSpacer_2 = new QSpacerItem(20, 40, QSizePolicy::Minimum, QSizePolicy::Expanding);

    gridLayout->addItem(verticalSpacer_2, 7, 0, 1, 1);

    retranslateUi(dummy_form);

    QMetaObject::connectSlotsByName(dummy_form);
  }  // setupUi

  void retranslateUi(QWidget *dummy_form)
  {
    dummy_form->setWindowTitle(QApplication::translate("dummy_form", "Calculator Form", 0, QApplication::UnicodeUTF8));
    label_2->setText(QApplication::translate("dummy_form", "Kinect", 0, QApplication::UnicodeUTF8));
    label->setText(QApplication::translate("dummy_form", "Subscribe to Topic", 0, QApplication::UnicodeUTF8));
    checkBox->setText(QApplication::translate("dummy_form", "Printf pixel and projected point coordinates", 0,
                                              QApplication::UnicodeUTF8));
    checkBox_4->setText(QApplication::translate("dummy_form", "Enable point validaton and verify jump range", 0,
                                                QApplication::UnicodeUTF8));
    label_4->setText(QApplication::translate("dummy_form", "Moving average:", 0, QApplication::UnicodeUTF8));
    radioButton->setText(QApplication::translate("dummy_form", "Simple", 0, QApplication::UnicodeUTF8));
    radioButton_2->setText(QApplication::translate("dummy_form", "Linear", 0, QApplication::UnicodeUTF8));
    radioButton_3->setText(QApplication::translate("dummy_form", "Exponential", 0, QApplication::UnicodeUTF8));
    label_7->setText(QApplication::translate("dummy_form", "Window size", 0, QApplication::UnicodeUTF8));
    label_8->setText(QApplication::translate("dummy_form", "nPoints", 0, QApplication::UnicodeUTF8));
    label_3->setText(QApplication::translate("dummy_form", "Smartphone", 0, QApplication::UnicodeUTF8));
    checkBox_2->setText(QApplication::translate("dummy_form", "Log in fixed rotation offset smartphone <-> robotbase",
                                                0, QApplication::UnicodeUTF8));
    label_5->setText(QApplication::translate("dummy_form", "Robot", 0, QApplication::UnicodeUTF8));
    checkBox_3->setText(QApplication::translate("dummy_form", "show URDF Model", 0, QApplication::UnicodeUTF8));
    checkBox_5->setText(QApplication::translate("dummy_form", "use IK-Solver", 0, QApplication::UnicodeUTF8));
    label_9->setText(QApplication::translate("dummy_form", " Joint 1", 0, QApplication::UnicodeUTF8));
    label_6->setText(QApplication::translate("dummy_form", "rad1", 0, QApplication::UnicodeUTF8));
    label_12->setText(QApplication::translate("dummy_form", " Joint 2", 0, QApplication::UnicodeUTF8));
    label_15->setText(QApplication::translate("dummy_form", "rad2", 0, QApplication::UnicodeUTF8));
    label_13->setText(QApplication::translate("dummy_form", " Joint 3", 0, QApplication::UnicodeUTF8));
    label_16->setText(QApplication::translate("dummy_form", "rad3", 0, QApplication::UnicodeUTF8));
    label_14->setText(QApplication::translate("dummy_form", " Joint 4", 0, QApplication::UnicodeUTF8));
    label_17->setText(QApplication::translate("dummy_form", "rad4", 0, QApplication::UnicodeUTF8));
    label_10->setText(QApplication::translate("dummy_form", " Joint 5 ", 0, QApplication::UnicodeUTF8));
    label_18->setText(QApplication::translate("dummy_form", "rad5", 0, QApplication::UnicodeUTF8));
    label_11->setText(QApplication::translate("dummy_form", " Joint 6 ", 0, QApplication::UnicodeUTF8));
    label_19->setText(QApplication::translate("dummy_form", "rad6", 0, QApplication::UnicodeUTF8));
    label_20->setText(QApplication::translate("dummy_form", "Command type:", 0, QApplication::UnicodeUTF8));
    radioButton_4->setText(QApplication::translate("dummy_form", "Joint", 0, QApplication::UnicodeUTF8));
    radioButton_5->setText(QApplication::translate("dummy_form", "Position", 0, QApplication::UnicodeUTF8));
    label_21->setText(QApplication::translate("dummy_form", "Robot IP4 / Port", 0, QApplication::UnicodeUTF8));
    pushButton->setText(QApplication::translate("dummy_form", "Connect", 0, QApplication::UnicodeUTF8));
  }  // retranslateUi
};

namespace Ui
{
class dummy_form : public Ui_dummy_form
{
};
}  // namespace Ui

QT_END_NAMESPACE

#endif  // DOTNECT_WINDOW_H
