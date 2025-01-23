#ifndef PROGRESS_BAR_HPP
#define PROGRESS_BAR_HPP

#include <iomanip>
#include <iostream>
#include <string>

class ProgressBar
{
  public:
    ProgressBar(size_t total, size_t width = 50)
        : total_(total)
        , width_(width)
        , current_(0)
    {
    }

    void update(size_t progress)
    {
        current_ = progress;
        float ratio = static_cast<float>(current_) / static_cast<float>(total_);
        size_t filled_width = static_cast<size_t>(ratio * width_);

        std::string bar(filled_width, '=');
        if (filled_width < width_)
        {
            bar += '>';
            bar.resize(width_, ' ');
        }

        std::cout << "\r[" << bar << "] " << std::setw(3)
                  << static_cast<int>(ratio * 100) << "% (" << current_ << "/"
                  << total_ << ")" << std::flush;

        if (current_ >= total_)
        {
            std::cout << std::endl;
        }
    }

    void increment(size_t step = 1) { update(current_ + step); }

  private:
    size_t total_;
    size_t width_;
    size_t current_;
};

#endif // PROGRESS_BAR_HPP
