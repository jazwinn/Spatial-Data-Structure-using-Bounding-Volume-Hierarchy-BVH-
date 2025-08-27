/**
 * @file CS350Loader.hpp
 * @author Eder Beldad (eder.b1@digipen.edu)
 * @co-author Jaz Winn Ng (jazwinn.ng@digipen.edu)
 * @brief Utilities to load custom asset data. Does not create graphics assets.
 * @date 2023-05-19
 * 
 * @copyright Copyright (c) 2023
 * 
 */
#ifndef CS350LOADER_CPP
#define CS350LOADER_CPP

#include "cs350_loader.hpp"
#include "utils.hpp"

#include <sstream>

namespace CS350 {

	//Set Initial fixed length for model binary file
	constexpr size_t signatureLength = 5;
	constexpr size_t countsPresentsLength = 11;
	constexpr size_t totalFileInitLength = signatureLength + countsPresentsLength;

	CS350PrimitiveData LoadCS350Binary(std::string const& file)
	{
	
		std::string binary = CS350::LoadFileBinary(file);

		// retrieve signature and check validity
		std::string signature = binary.substr(0, signatureLength);
		if (signature != "CS350") {
			throw std::runtime_error("LoadCS350Binary: Signature not CS350");
		}

		int vertexCount = 0;
		int indexCount = 0;
		bool positionPresent = true;
		bool normalPresent = true;
		bool uvsPresent = true;

		// Extract counts and data present from the binary
		UnpackBinary( binary , signatureLength ,vertexCount, indexCount, positionPresent, normalPresent, uvsPresent);

		
		
		CS350PrimitiveData data{};
		data.positions.reserve(vertexCount);
		
		// since asset only contains position
		if (positionPresent) {
			for (int n{}; n < vertexCount; n++) {

				vec3 position{};
				//extract position
				UnpackBinary(binary, totalFileInitLength + (n * sizeof(vec3)),  position);

				data.positions.push_back(position);
			}
		}

		//Creaet bounding volume
		Aabb aabb(data.positions.data(), data.positions.size());

		data.bvMax = aabb.max;
		data.bvMin = aabb.min;


		return data;
	}
	std::vector<CS350SceneObject> LoadCS350Scene(std::string const& file)
	{
		// File existence
		if (!std::filesystem::exists(file)) {
			throw std::runtime_error(fmt::format("Could not find file {}", file));
		}
		// Open
		std::ifstream fs(file);
		if (!fs.is_open()) {
			throw std::runtime_error(fmt::format("Could not open file {}", file));
		}


		std::vector<CS350SceneObject> sceneObjects;
		std::string line;
		while (std::getline(fs, line)) {
			
			CS350::CS350SceneObject object{0, mat4()};
			object.primitiveIndex = std::stoi(line);
			
			if (!std::getline(fs, line)) {
				break;
			}

			std::istringstream is(line);
			//read textfile and assign to variable
			GenericVecRead(is, object.m2w);

			sceneObjects.emplace_back(object);
		}



		return sceneObjects;
	}
}

#endif // CS350LOADER_CPP
