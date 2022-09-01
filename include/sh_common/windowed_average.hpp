#pragma once

#include <vector>
#include <queue>
#include <algorithm>
#include <functional>

namespace sh {

// A helper class to apply a windowed average to N parallel channels of data of
// type \a DataT
template <typename DataT>
struct WindowedAverage
{
protected:
    // The current number of elements in each channel of the window
    size_t current_size;
    // The current totals of each channel of the window
    std::vector<DataT> current_totals;
    // The current data in each channel of the window
    std::vector<std::queue<DataT>> current_data;

public:
    // The number of parallel windows to maintain
    const size_t num_channels;
    // The maximum number of elements in each channel of the window
    const size_t max_size;

    WindowedAverage(const size_t num_channels,
                    const size_t max_size) :
        num_channels(num_channels),
        max_size(max_size),
        current_size(0)
    {
        current_totals.resize(num_channels);
        current_data.resize(num_channels);
    }
    virtual ~WindowedAverage()
    {
    }

    // Initialize the current data.
    void reset(const DataT& init_value)
    {
        for (auto iter = current_data.begin(); iter != current_data.end(); ++iter)
        {
            while (!iter->empty())
            {
                iter->pop();
            }
        }
        current_totals.resize(num_channels, init_value);
        current_size = 0;
    }

    // Whether the window is full (\a current_size == \a max_size)
    bool isFull() const
    {
        return max_size == current_size;
    }

    // Evaluate the current windowed average.
    // @param avg Overwritten with the current windowed average.
    void eval(std::vector<DataT>& avg) const
    {
        avg.resize(num_channels);
        std::transform(
            current_totals.cbegin(),
            current_totals.cend(),
            avg.begin(),
            [this](const DataT& d) { return static_cast<DataT>(d / current_size); }
        );
        return avg;
    }

    // Push a new sample into the window. If the window is already full,
    // discard its oldest sample. If the input data is of the wrong size, then
    // do nothing and return false.
    // @param new_data A new sample for each channel of the window.
    // @return Whether the data could be pushed (true), or not (false).
    bool push(const std::vector<DataT>& new_data)
    {
        if (num_channels != new_data.size())
        {
            return false;
        }

        // Overwrite the current totals with the sum of the current totals and
        // the new data in each channel, and add this data to the queue
        for (size_t i = 0; i < num_channels; i++)
        {
            current_totals[i] += new_data[i];
            current_data[i].push(new_data[i]);
        }

        current_size++;

        // Pop data until we do not exceed the max size (in actuality this loop
        // should only be entered 0 or 1 times)
        while (current_size > max_size)
        {
            for (size_t i = 0; i < num_channels; i++)
            {
                current_totals[i] -= current_data[i].front();
                current_data[i].pop();
            }
            current_size--;
        }
    }

    // First push a new sample and then evaluate the current windowed average.
    // @param new_data A new sample for each channel of the window.
    // @param avg Overwritten with the current windowed average.
    // @return Whether the data could be pushed (true), or not (false).
    bool push_and_eval(const std::vector<DataT>& new_data, std::vector<DataT>& avg)
    {
        const bool rc = push(new_data);
        avg = eval();
        return rc;
    }
};

}
