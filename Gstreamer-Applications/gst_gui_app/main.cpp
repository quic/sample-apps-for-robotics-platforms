/*
* Copyright (c) 2020 Qualcomm Innovation Center, Inc.  All Rights Reserved.
*
* SPDX-License-Identifier: BSD-3-Clause-Clear
*/

#include <QApplication>

#include "window.h"

int main(int argc, char *argv[])
{
    QApplication app(argc, argv);
    Window window;
    window.show();
    return app.exec();
}
