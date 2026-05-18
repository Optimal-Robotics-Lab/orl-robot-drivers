#pragma once

#include <Eigen/Dense>
#include <stdexcept>
#include <string>
#include <cstddef>


/**
 * @brief First-order exponential smoothing filter.
 */
template <std::size_t ActionSize>
class FirstOrderFilter {
    public:
        using ActionVector = Eigen::Vector<float, ActionSize>;
        using ObservationVector = Eigen::Vector<float, ActionSize>;

    private:
        float alpha;
        ActionVector last_filtered_action{ActionVector::Zero()};

    public:
        explicit FirstOrderFilter(float alpha) : alpha(alpha) {
            if (this->alpha < 0.0f || this->alpha > 1.0f) {
                throw std::invalid_argument("[FirstOrderFilter] Alpha must be between 0.0 and 1.0.");
            }
            this->reset();
        }

        void reset() {
            this->last_filtered_action.setZero();
        }

        [[nodiscard]] ActionVector apply(const ActionVector& action) {
            this->last_filtered_action = this->alpha * action + (1.0f - this->alpha) * this->last_filtered_action;
            return this->last_filtered_action;
        }

        [[nodiscard]] ObservationVector get_observation() const {
            return this->last_filtered_action;
        }
        
        [[nodiscard]] static constexpr std::size_t get_actions_size() {
            return ActionSize;
        }

        [[nodiscard]] static constexpr std::size_t get_observation_size() {
            return ActionSize;
        }
};

/**
 * @brief Second-order filter.
 */
template <std::size_t ActionSize>
class SecondOrderFilter {
    public:
        using ActionVector = Eigen::Vector<float, ActionSize>;
        using ObservationVector = Eigen::Vector<float, 2 * ActionSize>;

    private:
        float b0, b1, b2, a1, a2;
        ActionVector x_t1, x_t2, y_t1, y_t2;

    public:
        explicit SecondOrderFilter(float b0, float b1, float b2, float a1, float a2)
            : b0(b0), b1(b1), b2(b2), a1(a1), a2(a2) {
            this->reset();
        }

        void reset() {
            this->x_t1.setZero();
            this->x_t2.setZero();
            this->y_t1.setZero();
            this->y_t2.setZero();
        }

        [[nodiscard]] ActionVector apply(const ActionVector& action) {
            ActionVector filtered_action = this->b0 * action 
                                            + this->b1 * this->x_t1 
                                            + this->b2 * this->x_t2 
                                            - this->a1 * this->y_t1 
                                            - this->a2 * this->y_t2;
            
            // Shift history states
            this->x_t2 = this->x_t1;
            this->x_t1 = action;
            this->y_t2 = this->y_t1;
            this->y_t1 = filtered_action;

            return filtered_action;
        }

        [[nodiscard]] ObservationVector get_observation() const {
            auto target_velocity = this->y_t1 - this->y_t2;
            const auto& previous_filtered_action = this->y_t1;
            ObservationVector observation;
            observation << previous_filtered_action, target_velocity;
            return observation;
        }
        
        [[nodiscard]] static constexpr std::size_t get_actions_size() {
            return ActionSize;
        }

        [[nodiscard]] static constexpr std::size_t get_observation_size() {
            return 2 * ActionSize;
        }
};

/**
 * @brief No-Op filter that passes actions through unmodified.
 */
template <std::size_t ActionSize>
class NoFilter {
    public:
        using ActionVector = Eigen::Vector<float, ActionSize>;
        using ObservationVector = Eigen::Vector<float, 0>;

    public:
        explicit NoFilter() = default;

        void reset() { }

        [[nodiscard]] ActionVector apply(const ActionVector& action) {
            return action;
        }

        [[nodiscard]] ObservationVector get_observation() const {
            return ObservationVector();
        }
        
        [[nodiscard]] static constexpr std::size_t get_actions_size() {
            return ActionSize;
        }

        [[nodiscard]] static constexpr std::size_t get_observation_size() {
            return 0;
        }
};