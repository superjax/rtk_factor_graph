#include <chrono>
#include <cmath>
#include <string>
#include <vector>

namespace mc {
namespace utils {

class ProgressBar
{
 public:
    ProgressBar();
    ProgressBar(int total, int barwidth);

    ~ProgressBar();

    void init(int total, int barwidth);

    void set_theme_line();
    void set_theme_circle();
    void set_theme_braille();
    void set_theme_braille_spin();
    void set_theme_solid();

    void print(int completed, double t = NAN);

    void finished();

 private:
    std::string ms_to_stamp(int ms);

    bool initialized_;
    int barwidth_;
    int total_;

    double rt_alpha_;
    double rt_rate_;
    double t_last_;
    double elapsed_last_;
    int last_completed_;
    double t_start_;
    double t_end_;
    std::vector<const char *> bars_;

    std::chrono::system_clock::time_point start_time_;
    std::chrono::system_clock::time_point last_print_time_;
};

}  // namespace utils
}  // namespace mc
