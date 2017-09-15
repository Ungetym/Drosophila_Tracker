#ifndef SYSTEM_STATE_H
#define SYSTEM_STATE_H

#include <QObject>
#include <QElapsedTimer>
#include <QPixmap>
#include <opencv2/opencv.hpp>
#include "rjmcmc_target.h"

//structs used by simple tracker
struct contour{
    std::vector<cv::Point> contour_points;
    cv::Rect pos_in_binary;
    cv::Mat binary;
};

struct target{
    std::vector<int> ids;
    contour ct;
    std::vector<int> succ;
    std::vector<int> pred;
    int frame_nr;
    bool marked = false;
    //save the min/max possible number of larvae contained in the contour
    int min_larvae, max_larvae;

};

struct collision{
    int start_frame;
    int end_frame;
    std::vector<std::vector<target>> targets;
};

struct simpleTrackerState{
    std::vector<std::vector<target>> tracked_targets;
    cv::Mat last_binary, current_binary;
    std::vector<contour> last_contours, current_contours;
    QStringList file_list;
};

struct IO{
    QString seq_dir, coll_dir, output_dir;
    //colors for output images
    std::vector<cv::Scalar> colors;
};

struct RJMCMCState{
    //Parameters the user can set via gui
    int B = 150; //number of burn-in samples
    int M = 1; //thin-out factor (studies suggest it is irrelevant)
    int N = 1500; //final number of samples
    float fps = 10.0; //frame rate of sequences
    float scaling_factor = 1.0; //enables up- or downscaling of input sequences
    float a_threshold = 0.75; //modify in order to reach ca. 25% accepted samples

    float current_rate_of_accepted = 0.0; //saves the rate of accepted samples

    //internal parameters
    float avg_spine_length = 0.0;
    float avg_larva_area = 0.0;
    float max_dist; //= avg_spine_length/4.0 is the maximum distance we allow between two spine mid points
    float min_dist; //= avg_spine_length/10.0 is the minimum distance we allow between two spine mid points

    cv::Mat current_image; //saved for evaluation and head selection
    sample best_sample;

    bool first_frame=true;
};

struct HeadSelectionState{
    //in order to evaluate the same set repeatedly, the initial head positions can be saved (only useful when the head points are set manually)
    std::string headfile_path;
    std::vector<cv::Point2f> headpoints;
};

struct EvaluationState{
    bool manual_evaluation=true;
    bool sequence_evaluated = true;
    float num_of_frames;
    std::vector<QPixmap> current_sequence;
    QString current_results_dir;
    QString current_sequence_dir;
    QString current_result_sequence_dir;
    size_t* current_idx;
    QElapsedTimer timer;
    qint64 time_needed = 0;
    float num_of_current_ids;
    float correct_ids = 0.0;
    float wrong_ids = 0.0;
    float sum_ids = 0.0;
};

class System_State : public QObject{
    Q_OBJECT

public:
    System_State();

    simpleTrackerState simple;
    IO io;
    RJMCMCState sampling;
    HeadSelectionState heads;
    EvaluationState eval;

};

#endif // SYSTEM_STATE_H
