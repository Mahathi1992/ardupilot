#ifndef SimpleMovingAverage_H
#define SimpleMovingAverage_H

template <uint8_t N, class input_t, class sum_t>
class SimpleMovingAverage {
  public:
    input_t operator()(input_t input) {
        sum -= previousInputs[index];
        sum += input;
        previousInputs[index] = input;
        if (++index == N)
            index = 0;
        return (sum + (N / 2)) / N;
    }
    

  private:
    uint8_t index             = 0;
    input_t previousInputs[N] = {};
    sum_t sum                 = 0;
};

#endif
