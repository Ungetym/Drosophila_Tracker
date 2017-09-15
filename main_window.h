#ifndef MAIN_WINDOW_H
#define MAIN_WINDOW_H

#include <QMainWindow>
#include <opencv2/opencv.hpp>

#include "consts.h"
#include "simple_tracker.h"
#include "rjmcmc_tracker.h"
#include "system_state.h"
#include "contour_extractor.h"

namespace Ui {
class Main_Window;
}

class Main_Window : public QMainWindow
{
    Q_OBJECT

public:
    explicit Main_Window(QWidget *parent = 0);
    ~Main_Window();

private slots:
    //handles accept/reject buttons for manual evaluation
    void manualEvaluation();
    //sets text to the logger console
    void setLogText(QString msg, int type);
    //opens file dialog to enable the user to select a dir
    void selectDir();
    //opens directory associated with button
    void openDir();
    //get and check parameters from gui
    void applyParameters();
    //starts collision solver for extracted collisions found in coll_dir
    void collisionSolving();

private:
    Ui::Main_Window *ui;

    System_State state;
    Simple_Tracker simple_tracker;
    RJMCMC_Tracker rjmcmc_tracker;
    contour_extractor ct_extractor;

    //draws given sample onto image and saves the result
    void drawTargets(sample* sample, Mat *image, float zoom, std::string const& image_output_path, size_t image_idx);

signals:
    void log(QString msg, int type);
    void progress(float val);
};

#endif // MAIN_WINDOW_H
