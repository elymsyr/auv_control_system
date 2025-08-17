#ifndef MODEL_INFERENCE_H
#define MODEL_INFERENCE_H

#include <string>
#include <vector>
#include <opencv2/dnn.hpp> // Main OpenCV DNN include

class ModelInference {
public:
    /**
     * @brief Default constructor.
     */
    ModelInference();

    /**
     * @brief Destructor.
     */
    ~ModelInference();

    /**
     * @brief Loads the neural network model from an ONNX file.
     * @param modelPath The file path to the .onnx model.
     * @return True if loading was successful, false otherwise.
     */
    bool loadModel(const std::string& modelPath);

    /**
     * @brief Loads the normalization/denormalization parameters from a JSON file.
     * @param scalerPath The file path to the .json scaler data.
     * @return True if loading was successful, false otherwise.
     */
    bool loadScalers(const std::string& scalerPath);

    /**
     * @brief Runs inference on the provided input data.
     * @param rawInputData A vector of floats representing the raw input to the model.
     * @return A vector of floats representing the model's output.
     * @throws std::runtime_error if the model or scalers are not loaded.
     * @throws std::invalid_argument if the input data has an incorrect size.
     */
    std::vector<float> runInference(const std::vector<float>& rawInputData);

private:
    // The OpenCV DNN network object.
    cv::dnn::Net net;

    // Scaling parameters for normalization.
    std::vector<float> x_mean, x_std; // For input data
    std::vector<float> y_mean, y_std; // For output data
    bool scalers_loaded;

    /**
     * @brief Normalizes input data using pre-loaded mean and std deviation.
     */
    std::vector<float> normalize(const std::vector<float>& data, const std::vector<float>& mean, const std::vector<float>& std);

    /**
     * @brief Denormalizes output data using pre-loaded mean and std deviation.
     */
    std::vector<float> denormalize(const std::vector<float>& data, const std::vector<float>& mean, const std::vector<float>& std);
};

#endif // MODEL_INFERENCE_H