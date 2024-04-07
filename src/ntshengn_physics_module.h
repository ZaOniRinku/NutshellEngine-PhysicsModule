#pragma once
#include "../Common/module_interfaces/ntshengn_physics_module_interface.h"
#include "../Module/utils/ntshengn_module_defines.h"
#include "../external/Jolt/Jolt/Jolt.h"
#include "../external/Jolt/Jolt/RegisterTypes.h"
#include "../external/Jolt/Jolt/Core/Factory.h"
#include "../external/Jolt/Jolt/Core/TempAllocator.h"
#include "../external/Jolt/Jolt/Core/JobSystemThreadPool.h"
#include "../external/Jolt/Jolt/Physics/PhysicsSettings.h"
#include "../external/Jolt/Jolt/Physics/PhysicsSystem.h"
#include "../external/Jolt/Jolt/Physics/Collision/Shape/BoxShape.h"
#include "../external/Jolt/Jolt/Physics/Collision/Shape/SphereShape.h"
#include "../external/Jolt/Jolt/Physics/Collision/Shape/CapsuleShape.h"
#include "../external/Jolt/Jolt/Physics/Body/BodyCreationSettings.h"
#include "../external/Jolt/Jolt/Physics/Body/BodyActivationListener.h"
#include <thread>
#include <unordered_map>

namespace ObjectLayers {
	static constexpr JPH::ObjectLayer NON_MOVING = 0;
	static constexpr JPH::ObjectLayer MOVING = 1;
	static constexpr uint32_t NUM_LAYERS = 2;
};

class ObjectLayerPairFilterImpl : public JPH::ObjectLayerPairFilter {
public:
	virtual bool ShouldCollide(JPH::ObjectLayer inObject1, JPH::ObjectLayer inObject2) const override {
		switch (inObject1) {
		case ObjectLayers::NON_MOVING:
			return inObject2 == ObjectLayers::MOVING;

		case ObjectLayers::MOVING:
			return true;

		default:
			JPH_ASSERT(false);
			return false;
		}
	}
};

namespace BroadPhaseLayers {
	static constexpr JPH::BroadPhaseLayer NON_MOVING(0);
	static constexpr JPH::BroadPhaseLayer MOVING(1);
	static constexpr uint32_t NUM_LAYERS(2);
};

class BroadphaseLayerInterfaceImpl : public JPH::BroadPhaseLayerInterface {
public:
	BroadphaseLayerInterfaceImpl() {
		m_objectToBroadPhase[ObjectLayers::NON_MOVING] = BroadPhaseLayers::NON_MOVING;
		m_objectToBroadPhase[ObjectLayers::MOVING] = BroadPhaseLayers::MOVING;
	}

	virtual uint32_t GetNumBroadPhaseLayers() const override {
		return BroadPhaseLayers::NUM_LAYERS;
	}

	virtual JPH::BroadPhaseLayer GetBroadPhaseLayer(JPH::ObjectLayer inLayer) const override {
		JPH_ASSERT(inLayer < ObjectLayers::NUM_LAYERS);
		return m_objectToBroadPhase[inLayer];
	}

#if defined(JPH_EXTERNAL_PROFILE) || defined(JPH_PROFILE_ENABLED)
	virtual const char* GetBroadPhaseLayerName(JPH::BroadPhaseLayer inLayer) const override {
		switch (static_cast<JPH::BroadPhaseLayer::Type>(inLayer)) {
			case static_cast<JPH::BroadPhaseLayer::Type>(BroadPhaseLayers::NON_MOVING):
				return "NON_MOVING";

			case static_cast<JPH::BroadPhaseLayer::Type>(BroadPhaseLayers::MOVING):
				return "MOVING";

			default:
				JPH_ASSERT(false);
				return "INVALID";
		}
	}
#endif

private:
	JPH::BroadPhaseLayer m_objectToBroadPhase[ObjectLayers::NUM_LAYERS];
};

class ObjectVsBroadPhaseLayerFilterImpl : public JPH::ObjectVsBroadPhaseLayerFilter {
public:
	virtual bool ShouldCollide(JPH::ObjectLayer inObject1, JPH::BroadPhaseLayer inObject2) const override {
		switch (inObject1) {
		case ObjectLayers::NON_MOVING:
			return inObject2 == BroadPhaseLayers::MOVING;

		case ObjectLayers::MOVING:
			return true;

		default:
			JPH_ASSERT(false);
			return false;
		}
	}
};

class ContactListener : public JPH::ContactListener {
public:
	virtual JPH::ValidateResult OnContactValidate(const JPH::Body& inBody1, const JPH::Body& inBody2, JPH::RVec3Arg inBaseOffset, const JPH::CollideShapeResult& inCollisionResult) override {
		NTSHENGN_UNUSED(inBody1);
		NTSHENGN_UNUSED(inBody2);
		NTSHENGN_UNUSED(inBaseOffset);
		NTSHENGN_UNUSED(inCollisionResult);
		NTSHENGN_MODULE_INFO("Contact validate callback.");

		return JPH::ValidateResult::AcceptAllContactsForThisBodyPair;
	}

	virtual void OnContactAdded(const JPH::Body& inBody1, const JPH::Body& inBody2, const JPH::ContactManifold& inManifold, JPH::ContactSettings& ioSettings) override {
		NTSHENGN_UNUSED(inBody1);
		NTSHENGN_UNUSED(inBody2);
		NTSHENGN_UNUSED(inManifold);
		NTSHENGN_UNUSED(ioSettings);
		NTSHENGN_MODULE_INFO("A contact was added.");
	}

	virtual void OnContactPersisted(const JPH::Body& inBody1, const JPH::Body& inBody2, const JPH::ContactManifold& inManifold, JPH::ContactSettings& ioSettings) override {
		NTSHENGN_UNUSED(inBody1);
		NTSHENGN_UNUSED(inBody2);
		NTSHENGN_UNUSED(inManifold);
		NTSHENGN_UNUSED(ioSettings);
		NTSHENGN_MODULE_INFO("A contact was persisted.");
	}

	virtual void OnContactRemoved(const JPH::SubShapeIDPair& inSubShapePair) override {
		NTSHENGN_UNUSED(inSubShapePair);
		NTSHENGN_MODULE_INFO("A contact was removed.");
	}
};

class BodyActivationListener : public JPH::BodyActivationListener {
public:
	virtual void OnBodyActivated(const JPH::BodyID& inBodyID, uint64_t inBodyUserData) override {
		NTSHENGN_UNUSED(inBodyID);
		NTSHENGN_UNUSED(inBodyUserData);
		NTSHENGN_MODULE_INFO("A body got activated.");
	}

	virtual void OnBodyDeactivated(const JPH::BodyID& inBodyID, uint64_t inBodyUserData) override {
		NTSHENGN_UNUSED(inBodyID);
		NTSHENGN_UNUSED(inBodyUserData);
		NTSHENGN_MODULE_INFO("A body got deactivated.");
	}
};

namespace NtshEngn {

	class PhysicsModule : public PhysicsModuleInterface {
	public:
		PhysicsModule() : PhysicsModuleInterface("NutshellEngine Default Physics Module") {}

		void init();
		void update(double dt);
		void destroy();

		// Returns an IntersectionInformation structure containing information about the intersection
		IntersectionInformation intersect(const ColliderShape* shape1, const ColliderShape* shape2);

		// Returns a RaycastInformation structure containing information about the raycast
		RaycastInformation raycast(const Math::vec3& rayOrigin, const Math::vec3& rayDirection, float tMin, float tMax, const ColliderShape* shape);
		// Returns a list of RaycastInformation structures containing information about the hit entities
		std::vector<std::pair<Entity, RaycastInformation>> raycastAll(const Math::vec3& rayOrigin, const Math::vec3& rayDirection, float tMin, float tMax);

	public:
		const ComponentMask getComponentMask() const;

		void onEntityComponentAdded(Entity entity, Component componentID);
		void onEntityComponentRemoved(Entity entity, Component componentID);

	private:
		Math::vec3 getCenter(const ColliderShape* shape);
		Math::vec3 getCenter(const ColliderSphere* sphere);
		Math::vec3 getCenter(const ColliderBox* box);
		Math::vec3 getCenter(const ColliderCapsule* capsule);

		void transform(ColliderShape* shape, const Math::vec3& translation, const Math::vec3& rotation, const Math::vec3& scale);
		void transform(ColliderBox* box, const Math::vec3& translation, const Math::vec3& rotation, const Math::vec3& scale);
		void transform(ColliderSphere* sphere, const Math::vec3& translation, const Math::vec3& rotation, const Math::vec3& scale);
		void transform(ColliderCapsule* capsule, const Math::vec3& translation, const Math::vec3& rotation, const Math::vec3& scale);

	private:
		const float m_deltaTime = 1.0f / 60.0f;
		float m_timeAccumulator = 0.0f;

		JPH::TempAllocatorImpl* m_tempAllocator;
		JPH::JobSystemThreadPool* m_jobSystem;

		BroadphaseLayerInterfaceImpl m_bpLayerInterface;
		ObjectVsBroadPhaseLayerFilterImpl m_objectVsBpLayerFilter;
		ObjectLayerPairFilterImpl m_objectVsObjectLayerFilter;
		JPH::PhysicsSystem* m_physicsSystem;

		BodyActivationListener m_bodyActivationListener;

		ContactListener m_contactListener;

		std::unordered_map<Entity, JPH::BodyID> m_entityToBodyID;
	};

}