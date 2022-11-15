#pragma once

#include <lenny/collision/Box.h>
#include <lenny/collision/Capsule.h>
#include <lenny/collision/Rectangle.h>
#include <lenny/collision/Sphere.h>

#include <functional>

namespace lenny::collision {

void drawGui(const char* label, const std::vector<Primitive::SPtr>& primitives);

typedef std::function<const Parent::SPtr(const std::string& parentDescription)> F_getParent;
void to_json(json& js, const std::vector<Primitive::SPtr>& primitives);
void from_json(const json& js, const F_getParent& f_getParent, std::vector<Primitive::SPtr>& primitives);

bool savePrimitivesToFile(const std::vector<Primitive::SPtr>& primitives, const std::string& filePath);
bool loadPrimitivesFromFile(std::vector<Primitive::SPtr>& primitives, const F_getParent& f_getParent, const char* filePath);

}  // namespace lenny::collision