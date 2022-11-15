#include <lenny/collision/Primitives.h>
#include <lenny/tools/Gui.h>

#include <fstream>
#include <magic_enum.hpp>

namespace lenny::collision {

void drawGui(const char* label, const std::vector<Primitive::SPtr>& primitives) {
    if (tools::Gui::I->TreeNode("World Collision Primitives")) {
        int iter = 0;
        for (Primitive::SPtr primitive : primitives)
            primitive->drawGui(std::to_string(iter++));

        tools::Gui::I->TreePop();
    }
}

void to_json(json& js, const std::vector<Primitive::SPtr>& primitives) {
    //Add primitives
    for (const Primitive::SPtr primitive : primitives) {
        json js_tmp;
        primitive->to_json(js_tmp);
        js.push_back(js_tmp);
    }
}

void from_json(const json& js, const F_getParent& f_getParent, std::vector<Primitive::SPtr>& primitives) {
    primitives.clear();
    enum PRIMITIVE_DESCRIPTIONS { Sphere, Capsule, Rectangle, Box };
    for (auto& js_tmp : js) {
        const std::string primitiveDescription = js_tmp.value("primitive-description", std::string());
        auto enum_descr = magic_enum::enum_cast<PRIMITIVE_DESCRIPTIONS>(primitiveDescription);
        if (!enum_descr.has_value()) {
            LENNY_LOG_WARNING("Unknown primitive description `%s`. Skipping...", primitiveDescription.c_str());
        } else {
            const std::string parentDescription = js_tmp.value("parent-description", std::string());
            switch (enum_descr.value()) {
                case Sphere: {
                    primitives.emplace_back(std::make_shared<collision::Sphere>(f_getParent(parentDescription), Eigen::Vector3d::Zero(), 0.1));
                    break;
                }
                case Capsule: {
                    primitives.emplace_back(
                        std::make_shared<collision::Capsule>(f_getParent(parentDescription), Eigen::Vector3d::Zero(), Eigen::Vector3d::Ones(), 0.1));
                    break;
                }
                case Rectangle: {
                    primitives.emplace_back(std::make_shared<collision::Rectangle>(f_getParent(parentDescription), Eigen::Vector3d::Zero(),
                                                                                   Eigen::QuaternionD::Identity(), Eigen::Vector2d::Ones(), 0.1));
                    break;
                }
                case Box: {
                    primitives.emplace_back(std::make_shared<collision::Box>(f_getParent(parentDescription), Eigen::Vector3d::Zero(),
                                                                             Eigen::QuaternionD::Identity(), Eigen::Vector3d::Ones(), 0.1));
                    break;
                }
                default:
                    LENNY_LOG_ERROR("Primitive description `%s` is not handled!", primitiveDescription.c_str());
            }
            primitives.back()->from_json(js_tmp);
        }
    }
}

bool savePrimitivesToFile(const std::vector<Primitive::SPtr>& primitives, const std::string& filePath) {
    //Check file extensions
    if (!tools::utils::checkFileExtension(filePath, "json"))
        LENNY_LOG_ERROR("File `%s` should be a `json` file", filePath.c_str())

    //Open file
    std::ofstream file(filePath);
    if (!file.is_open()) {
        LENNY_LOG_WARNING("File `%s` could not be opened\n", filePath.c_str());
        return false;
    }

    //To json
    json js;
    to_json(js, primitives);

    //Stream to file
    file << std::setw(2) << js << std::endl;

    //Close file
    file.close();

    //Wrap up
    LENNY_LOG_INFO("Successfully saved collision primitives into file `%s`", filePath.c_str());
    return true;
}

bool loadPrimitivesFromFile(std::vector<Primitive::SPtr>& primitives, const F_getParent& f_getParent, const char* fP) {
    //Initialize file path
    std::string filePath = fP ? std::string(fP) : tools::utils::browseFile();

    //Check file extensions
    if (!tools::utils::checkFileExtension(filePath, "json"))
        LENNY_LOG_ERROR("File `%s` should be a `json` file", filePath.c_str())

    //Open file
    std::ifstream file(filePath);
    if (!file.is_open())
        LENNY_LOG_ERROR("File `%s` could not be opened\n", filePath.c_str());

    //Load json from file
    json js;
    file >> js;

    //From json
    from_json(js, f_getParent, primitives);

    //Close file
    file.close();

    //Wrap up
    LENNY_LOG_INFO("Collision primitives successfully loaded from file `%s`", filePath.c_str());
    return true;
}

}  // namespace lenny::collision