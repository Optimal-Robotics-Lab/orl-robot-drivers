#pragma once

#include <Eigen/Dense>
#include <stdexcept>
#include <string>
#include <cstddef>

/**
 * @brief Abstract base class for action filters using compile-time dimensions.
 * @tparam ActionSize The size of the action vector output of the policy.
 */
template <std::size_t ActionSize>
class Filter {
    public:
        // Typedef to map exactly to your MotorVector<float>
        using ActionVector = Eigen::Matrix<float, ActionSize, 1>;

        virtual ~Filter() = default;

        // Delete copy semantics to prevent object slicing. Move is allowed.
        Filter(const Filter&) = delete;
        Filter& operator=(const Filter&) = delete;
        Filter(Filter&&) = default;
        Filter& operator=(Filter&&) = default;

        /**
        * @brief Resets the internal state of the filter.
        */
        virtual void reset() = 0;

        /**
        * @brief Applies the filter to the given action.
        */
        [[nodiscard]] virtual ActionVector apply(const ActionVector& action) = 0;

        /**
        * @brief Retrieves the filter history required for the RL observation.
        * @note Returns a dynamic VectorXf because different filters have different history sizes.
        */
        [[nodiscard]] virtual Eigen::VectorXf get_observation() const = 0;

        [[nodiscard]] virtual std::size_t get_observation_size() const {
            return get_observation().size();
        }
};

/**
 * @brief No-Op filter that passes actions through unmodified.
 */
template <std::size_t ActionSize>
class NoFilter : public Filter<ActionSize> {
    public:
        using ActionVector = typename Filter<ActionSize>::ActionVector;

        NoFilter() = default;

        void reset() override {}

        [[nodiscard]] ActionVector apply(const ActionVector& action) override {
            return action;
        }

        [[nodiscard]] Eigen::VectorXf get_observation() const override {
            return Eigen::VectorXf(0); // Empty observation
        }
};

/**
 * @brief First-order exponential smoothing filter.
 */
template <std::size_t ActionSize>
class FirstOrderFilter : public Filter<ActionSize> {
    public:
        using ActionVector = typename Filter<ActionSize>::ActionVector;

    private:
        float alpha_;
        ActionVector last_filtered_action_;

    public:
        explicit FirstOrderFilter(float alpha) : alpha_(alpha) {
            if (alpha_ < 0.0f || alpha_ > 1.0f) {
                throw std::invalid_argument("FirstOrderFilter - Alpha must be between 0.0 and 1.0.");
            }
            reset(); 
        }

        void reset() override {
            last_filtered_action_.setZero();
        }

        [[nodiscard]] ActionVector apply(const ActionVector& action) override {
            last_filtered_action_ = alpha_ * action + (1.0f - alpha_) * last_filtered_action_;
            return last_filtered_action_;
        }

        [[nodiscard]] Eigen::VectorXf get_observation() const override {
            return last_filtered_action_;
        }
};

/**
 * @brief Second-order digital filter.
 */
template <std::size_t ActionSize>
class SecondOrderFilter : public Filter<ActionSize> {
    public:
        using ActionVector = typename Filter<ActionSize>::ActionVector;

    private:
        float b0_, b1_, b2_, a1_, a2_;
        ActionVector x_t1_, x_t2_, y_t1_, y_t2_;

    public:
        SecondOrderFilter(float b0, float b1, float b2, float a1, float a2)
            : b0_(b0), b1_(b1), b2_(b2), a1_(a1), a2_(a2) {
            reset();
        }

        void reset() override {
            x_t1_.setZero();
            x_t2_.setZero();
            y_t1_.setZero();
            y_t2_.setZero();
        }

        [[nodiscard]] ActionVector apply(const ActionVector& action) override {
            ActionVector filtered_action = b0_ * action + b1_ * x_t1_ + b2_ * x_t2_ 
                                        - a1_ * y_t1_ - a2_ * y_t2_;

            // Shift history states
            x_t2_ = x_t1_;
            x_t1_ = action;
            
            y_t2_ = y_t1_;
            y_t1_ = filtered_action;

            return filtered_action;
        }

        [[nodiscard]] Eigen::VectorXf get_observation() const override {
            ActionVector target_velocity = y_t1_ - y_t2_;
            
            Eigen::VectorXf obs(ActionSize * 2);
            obs << y_t1_, target_velocity;
            
            return obs;
        }
};
