#include "ntshengn_physics_module.h"
#include "../Module/utils/ntshengn_dynamic_library.h"
#include "../Common/utils/ntshengn_defines.h"
#include "../Common/utils/ntshengn_enums.h"

void NtshEngn::PhysicsModule::init() {
	JPH::RegisterDefaultAllocator();
	
	JPH::Factory::sInstance = new JPH::Factory();

	JPH::RegisterTypes();

	m_tempAllocator = new JPH::TempAllocatorImpl(100 * 1024 * 1024);
	m_jobSystem = new JPH::JobSystemThreadPool(JPH::cMaxPhysicsJobs, JPH::cMaxPhysicsBarriers, std::thread::hardware_concurrency() - 1);

	const uint32_t maxBodies = MAX_ENTITIES;
	const uint32_t numBodyMutexes = 0;
	const uint32_t maxBodyPairs = 65536;
	const uint32_t maxContactConstraints = 10240;
	m_physicsSystem = new JPH::PhysicsSystem();
	m_physicsSystem->Init(maxBodies, numBodyMutexes, maxBodyPairs, maxContactConstraints, m_bpLayerInterface, m_objectVsBpLayerFilter, m_objectVsObjectLayerFilter);
	m_physicsSystem->SetBodyActivationListener(&m_bodyActivationListener);
	m_physicsSystem->SetContactListener(&m_contactListener);
}

void NtshEngn::PhysicsModule::update(double dt) {
	JPH::BodyInterface& bodyInterface = m_physicsSystem->GetBodyInterface();

	m_timeAccumulator += static_cast<float>(dt);
	const int nbIterations = static_cast<int>(std::floor(m_timeAccumulator / m_deltaTime));
	m_timeAccumulator -= static_cast<float>(nbIterations) * m_deltaTime;

	if (nbIterations > 0) {
		for (const auto& entityAndBodyID : m_entityToBodyID) {
			Entity entity = entityAndBodyID.first;
			JPH::BodyID bodyID = entityAndBodyID.second;

			const Transform& entityTransform = ecs->getComponent<Transform>(entity);
			const Rigidbody& rigidbody = ecs->getComponent<Rigidbody>(entity);

			bodyInterface.SetGravityFactor(bodyID, rigidbody.isAffectedByConstants ? 1.0f : 0.0f);
			bodyInterface.SetLinearVelocity(bodyID, JPH::Vec3(rigidbody.linearVelocity.x, rigidbody.linearVelocity.y, rigidbody.linearVelocity.z));
			bodyInterface.SetAngularVelocity(bodyID, JPH::Vec3(rigidbody.angularVelocity.x, rigidbody.angularVelocity.y, rigidbody.angularVelocity.z));
			bodyInterface.SetFriction(bodyID, rigidbody.staticFriction);
			bodyInterface.SetObjectLayer(bodyID, rigidbody.isStatic ? ObjectLayers::NON_MOVING : ObjectLayers::MOVING);
			bodyInterface.SetMotionType(bodyID, rigidbody.isStatic ? JPH::EMotionType::Static : JPH::EMotionType::Dynamic, JPH::EActivation::Activate);

			if (ecs->hasComponent<Collidable>(entity)) {
				const Collidable collidable = ecs->getComponent<Collidable>(entity);
				if (collidable.collider->getType() == ColliderShapeType::Box) {
					ColliderBox* colliderBox = static_cast<ColliderBox*>(collidable.collider.get());
					transform(colliderBox, entityTransform.position, entityTransform.rotation, entityTransform.scale);

					JPH::BoxShapeSettings boxShapeSettings(JPH::Vec3(colliderBox->halfExtent.x, colliderBox->halfExtent.y, colliderBox->halfExtent.z));
					JPH::ShapeSettings::ShapeResult boxShapeResult = boxShapeSettings.Create();
					JPH::ShapeRefC boxShape = boxShapeResult.Get();
					bodyInterface.SetShape(bodyID, boxShape, false, JPH::EActivation::Activate);
					const Math::quat rotationQuat = Math::eulerAnglesToQuat(colliderBox->rotation);
					bodyInterface.SetPositionAndRotation(bodyID, JPH::Vec3(colliderBox->center.x, colliderBox->center.y, colliderBox->center.z), JPH::Quat(rotationQuat.b, rotationQuat.c, rotationQuat.d, rotationQuat.a), JPH::EActivation::Activate);
				}
				else if (collidable.collider->getType() == ColliderShapeType::Sphere) {
					ColliderSphere* colliderSphere = static_cast<ColliderSphere*>(collidable.collider.get());
					transform(colliderSphere, entityTransform.position, entityTransform.rotation, entityTransform.scale);

					JPH::SphereShapeSettings sphereShapeSettings(colliderSphere->radius);
					JPH::ShapeSettings::ShapeResult sphereShapeResult = sphereShapeSettings.Create();
					JPH::ShapeRefC sphereShape = sphereShapeResult.Get();
					bodyInterface.SetShape(bodyID, sphereShape, false, JPH::EActivation::Activate);
					bodyInterface.SetPosition(bodyID, JPH::Vec3(colliderSphere->center.x, colliderSphere->center.y, colliderSphere->center.z), JPH::EActivation::Activate);
				}
				else if (collidable.collider->getType() == ColliderShapeType::Capsule) {
					ColliderCapsule* colliderCapsule = static_cast<ColliderCapsule*>(collidable.collider.get());
					transform(colliderCapsule, entityTransform.position, entityTransform.rotation, entityTransform.scale);

					JPH::CapsuleShapeSettings capsuleShapeSettings((colliderCapsule->tip - colliderCapsule->base).length() / 2.0f, colliderCapsule->radius);
					JPH::ShapeSettings::ShapeResult capsuleShapeResult = capsuleShapeSettings.Create();
					JPH::ShapeRefC capsuleShape = capsuleShapeResult.Get();
					bodyInterface.SetShape(bodyID, capsuleShape, false, JPH::EActivation::Activate);
					const Math::vec3 capsuleCenter = getCenter(colliderCapsule);
					Math::quat capsuleRotation;
					const Math::vec3 capsuleBaseDirection = Math::vec3(0.0f, 1.0f, 0.0f);
					const Math::vec3 capsuleDirection = Math::normalize(colliderCapsule->tip - colliderCapsule->base);
					if (capsuleDirection == Math::vec3(0.0f, -1.0f, 0.0f)) {
						capsuleRotation = Math::eulerAnglesToQuat(Math::vec3(Math::toRad(180.0f), 0.0f, 0.0f));
					}
					else {
						Math::vec3 baseCrossDirection = Math::cross(capsuleBaseDirection, capsuleDirection);
						float cosAngle = Math::dot(capsuleBaseDirection, capsuleDirection);
						const Math::mat4 skewSymmetricCrossProductMatrix = Math::mat4(Math::vec4(0.0f, baseCrossDirection.z, -baseCrossDirection.y, 0.0f),
							Math::vec4(-baseCrossDirection.z, 0.0f, baseCrossDirection.x, 0.0f),
							Math::vec4(baseCrossDirection.y, -baseCrossDirection.x, 0.0f, 0.0f),
							Math::vec4(0.0f, 0.0f, 0.0f, 0.0f));
						const Math::mat4 squaredSkewSymmetricCrossProductMatrix = Math::mat4(Math::vec4(0.0f, baseCrossDirection.z * baseCrossDirection.z, (-baseCrossDirection.y) * (-baseCrossDirection.y), 0.0f),
							Math::vec4((-baseCrossDirection.z) * (-baseCrossDirection.z), 0.0f, baseCrossDirection.x * baseCrossDirection.x, 0.0f),
							Math::vec4(baseCrossDirection.y * baseCrossDirection.y, (-baseCrossDirection.x) * (-baseCrossDirection.x), 0.0f, 0.0f),
							Math::vec4(0.0f, 0.0f, 0.0f, 0.0f));
						Math::mat4 capsuleRotationMatrix = Math::mat4() + skewSymmetricCrossProductMatrix + (squaredSkewSymmetricCrossProductMatrix * (1.0f / (1.0f + cosAngle)));
						capsuleRotation = Math::eulerAnglesToQuat(Math::rotationMatrixToEulerAngles(capsuleRotationMatrix));
					}
					bodyInterface.SetPositionAndRotation(bodyID, JPH::Vec3(capsuleCenter.x, capsuleCenter.y, capsuleCenter.z), JPH::Quat(capsuleRotation.b, capsuleRotation.c, capsuleRotation.d, capsuleRotation.a), JPH::EActivation::Activate);
				}
			}
		}

		m_physicsSystem->Update(m_deltaTime, nbIterations, m_tempAllocator, m_jobSystem);

		for (const auto& entityAndBodyID : m_entityToBodyID) {
			Entity entity = entityAndBodyID.first;
			JPH::BodyID bodyID = entityAndBodyID.second;

			Transform& entityTransform = ecs->getComponent<Transform>(entity);
			Rigidbody& rigidbody = ecs->getComponent<Rigidbody>(entity);

			const JPH::Vec3 bodyLinearVelocity = bodyInterface.GetLinearVelocity(bodyID);
			rigidbody.linearVelocity = Math::vec3(bodyLinearVelocity.GetX(), bodyLinearVelocity.GetY(), bodyLinearVelocity.GetZ());
			const JPH::Vec3 bodyAngularVelocity = bodyInterface.GetAngularVelocity(bodyID);
			rigidbody.angularVelocity = Math::vec3(bodyAngularVelocity.GetX(), bodyAngularVelocity.GetY(), bodyAngularVelocity.GetZ());

			if (ecs->hasComponent<Collidable>(entity)) {
				const Collidable& collidable = ecs->getComponent<Collidable>(entity);

				const JPH::Vec3 bodyPosition = bodyInterface.GetPosition(bodyID);
				entityTransform.position = Math::vec3(bodyPosition.GetX(), bodyPosition.GetY(), bodyPosition.GetZ()) - getCenter(collidable.collider.get());
				const JPH::Quat bodyRotation = bodyInterface.GetRotation(bodyID);
				const Math::vec3 bodyRotationEuler = Math::quatToEulerAngles(Math::quat(bodyRotation.GetW(), bodyRotation.GetX(), bodyRotation.GetY(), bodyRotation.GetZ()));
				if (collidable.collider->getType() == ColliderShapeType::Box) {
					ColliderBox* colliderBox = static_cast<ColliderBox*>(collidable.collider.get());

					entityTransform.rotation = bodyRotationEuler - colliderBox->rotation;
				}
				else if (collidable.collider->getType() == ColliderShapeType::Capsule) {
					ColliderCapsule* colliderCapsule = static_cast<ColliderCapsule*>(collidable.collider.get());

					const Math::vec3 capsuleBaseDirection = Math::vec3(0.0f, 1.0f, 0.0f);
					const Math::vec3 capsuleDirection = Math::normalize(colliderCapsule->tip - colliderCapsule->base);
					if (capsuleDirection == Math::vec3(0.0f, -1.0f, 0.0f)) {
						entityTransform.rotation = bodyRotationEuler - Math::vec3(Math::toRad(180.0f), 0.0f, 0.0f);
					}
					else {
						Math::vec3 baseCrossDirection = Math::cross(capsuleBaseDirection, capsuleDirection);
						float cosAngle = Math::dot(capsuleBaseDirection, capsuleDirection);
						const Math::mat4 skewSymmetricCrossProductMatrix = Math::mat4(Math::vec4(0.0f, baseCrossDirection.z, -baseCrossDirection.y, 0.0f),
							Math::vec4(-baseCrossDirection.z, 0.0f, baseCrossDirection.x, 0.0f),
							Math::vec4(baseCrossDirection.y, -baseCrossDirection.x, 0.0f, 0.0f),
							Math::vec4(0.0f, 0.0f, 0.0f, 0.0f));
						const Math::mat4 squaredSkewSymmetricCrossProductMatrix = Math::mat4(Math::vec4(0.0f, baseCrossDirection.z * baseCrossDirection.z, (-baseCrossDirection.y) * (-baseCrossDirection.y), 0.0f),
							Math::vec4((-baseCrossDirection.z) * (-baseCrossDirection.z), 0.0f, baseCrossDirection.x * baseCrossDirection.x, 0.0f),
							Math::vec4(baseCrossDirection.y * baseCrossDirection.y, (-baseCrossDirection.x) * (-baseCrossDirection.x), 0.0f, 0.0f),
							Math::vec4(0.0f, 0.0f, 0.0f, 0.0f));
						Math::mat4 capsuleRotationMatrix = Math::mat4() + skewSymmetricCrossProductMatrix + (squaredSkewSymmetricCrossProductMatrix * (1.0f / (1.0f + cosAngle)));

						entityTransform.rotation = bodyRotationEuler - Math::rotationMatrixToEulerAngles(capsuleRotationMatrix);
					}
				}
			}
		}
	}
}

void NtshEngn::PhysicsModule::destroy() {
	delete m_physicsSystem;

	delete m_tempAllocator;
	delete m_jobSystem;

	JPH::UnregisterTypes();

	delete JPH::Factory::sInstance;
	JPH::Factory::sInstance = nullptr;
}

NtshEngn::IntersectionInformation NtshEngn::PhysicsModule::intersect(const ColliderShape* shape1, const ColliderShape* shape2) {
	NTSHENGN_UNUSED(shape1);
	NTSHENGN_UNUSED(shape2);
	NTSHENGN_MODULE_FUNCTION_NOT_IMPLEMENTED();

	return IntersectionInformation();
}

NtshEngn::RaycastInformation NtshEngn::PhysicsModule::raycast(const Math::vec3& rayOrigin, const Math::vec3& rayDirection, float tMin, float tMax, const ColliderShape* shape) {
	NTSHENGN_UNUSED(rayOrigin);
	NTSHENGN_UNUSED(rayDirection);
	NTSHENGN_UNUSED(tMin);
	NTSHENGN_UNUSED(tMax);
	NTSHENGN_UNUSED(shape);
	NTSHENGN_MODULE_FUNCTION_NOT_IMPLEMENTED();

	return RaycastInformation();
}

std::vector<std::pair<NtshEngn::Entity, NtshEngn::RaycastInformation>> NtshEngn::PhysicsModule::raycastAll(const Math::vec3& rayOrigin, const Math::vec3& rayDirection, float tMin, float tMax) {
	NTSHENGN_UNUSED(rayOrigin);
	NTSHENGN_UNUSED(rayDirection);
	NTSHENGN_UNUSED(tMin);
	NTSHENGN_UNUSED(tMax);
	NTSHENGN_MODULE_FUNCTION_NOT_IMPLEMENTED();

	return std::vector<std::pair<Entity, RaycastInformation>>();
}

const NtshEngn::ComponentMask NtshEngn::PhysicsModule::getComponentMask() const {
	ComponentMask componentMask;
	componentMask.set(ecs->getComponentID<Rigidbody>());
	componentMask.set(ecs->getComponentID<Collidable>());

	return componentMask;
}

void NtshEngn::PhysicsModule::onEntityComponentAdded(Entity entity, Component componentID) {
	NTSHENGN_UNUSED(componentID);

	JPH::BodyInterface& bodyInterface = m_physicsSystem->GetBodyInterface();

	if (ecs->hasComponent<Rigidbody>(entity) && ecs->hasComponent<Collidable>(entity)) {
		const Transform& entityTransform = ecs->getComponent<Transform>(entity);
		const Rigidbody& rigidbody = ecs->getComponent<Rigidbody>(entity);
		const Collidable collidable = ecs->getComponent<Collidable>(entity);

		JPH::BodyCreationSettings bodySettings;
		bodySettings.mOverrideMassProperties = JPH::EOverrideMassProperties::MassAndInertiaProvided;
		bodySettings.mMassPropertiesOverride.mMass = rigidbody.mass;

		if (collidable.collider->getType() == ColliderShapeType::Box) {
			ColliderBox* colliderBox = static_cast<ColliderBox*>(collidable.collider.get());
			transform(colliderBox, entityTransform.position, entityTransform.rotation, entityTransform.scale);

			JPH::BoxShapeSettings boxShapeSettings(JPH::Vec3(colliderBox->halfExtent.x, colliderBox->halfExtent.y, colliderBox->halfExtent.z));
			JPH::ShapeSettings::ShapeResult boxShapeResult = boxShapeSettings.Create();
			JPH::ShapeRefC boxShape = boxShapeResult.Get();
			bodySettings.SetShape(boxShape);
			const Math::quat rotationQuat = Math::eulerAnglesToQuat(colliderBox->rotation);
			bodySettings.mPosition = JPH::Vec3(colliderBox->center.x, colliderBox->center.y, colliderBox->center.z);
			bodySettings.mRotation = JPH::Quat(rotationQuat.b, rotationQuat.c, rotationQuat.d, rotationQuat.a);
		}
		else if (collidable.collider->getType() == ColliderShapeType::Sphere) {
			ColliderSphere* colliderSphere = static_cast<ColliderSphere*>(collidable.collider.get());
			transform(colliderSphere, entityTransform.position, entityTransform.rotation, entityTransform.scale);

			JPH::SphereShapeSettings sphereShapeSettings(colliderSphere->radius);
			JPH::ShapeSettings::ShapeResult sphereShapeResult = sphereShapeSettings.Create();
			JPH::ShapeRefC sphereShape = sphereShapeResult.Get();
			bodySettings.SetShape(sphereShape);
			bodySettings.mPosition = JPH::Vec3(colliderSphere->center.x, colliderSphere->center.y, colliderSphere->center.z);
		}
		else if (collidable.collider->getType() == ColliderShapeType::Capsule) {
			ColliderCapsule* colliderCapsule = static_cast<ColliderCapsule*>(collidable.collider.get());
			transform(colliderCapsule, entityTransform.position, entityTransform.rotation, entityTransform.scale);

			JPH::CapsuleShapeSettings capsuleShapeSettings((colliderCapsule->tip - colliderCapsule->base).length() / 2.0f, colliderCapsule->radius);
			JPH::ShapeSettings::ShapeResult capsuleShapeResult = capsuleShapeSettings.Create();
			JPH::ShapeRefC capsuleShape = capsuleShapeResult.Get();
			bodySettings.SetShape(capsuleShape);
			const Math::vec3 capsuleCenter = getCenter(colliderCapsule);
			Math::quat capsuleRotation;
			const Math::vec3 capsuleBaseDirection = Math::vec3(0.0f, 1.0f, 0.0f);
			const Math::vec3 capsuleDirection = Math::normalize(colliderCapsule->tip - colliderCapsule->base);
			if (capsuleDirection == Math::vec3(0.0f, -1.0f, 0.0f)) {
				capsuleRotation = Math::eulerAnglesToQuat(Math::vec3(Math::toRad(180.0f), 0.0f, 0.0f));
			}
			else {
				Math::vec3 baseCrossDirection = Math::cross(capsuleBaseDirection, capsuleDirection);
				float cosAngle = Math::dot(capsuleBaseDirection, capsuleDirection);
				const Math::mat4 skewSymmetricCrossProductMatrix = Math::mat4(Math::vec4(0.0f, baseCrossDirection.z, -baseCrossDirection.y, 0.0f),
					Math::vec4(-baseCrossDirection.z, 0.0f, baseCrossDirection.x, 0.0f),
					Math::vec4(baseCrossDirection.y, -baseCrossDirection.x, 0.0f, 0.0f),
					Math::vec4(0.0f, 0.0f, 0.0f, 0.0f));
				const Math::mat4 squaredSkewSymmetricCrossProductMatrix = Math::mat4(Math::vec4(0.0f, baseCrossDirection.z * baseCrossDirection.z, (-baseCrossDirection.y) * (-baseCrossDirection.y), 0.0f),
					Math::vec4((-baseCrossDirection.z) * (-baseCrossDirection.z), 0.0f, baseCrossDirection.x * baseCrossDirection.x, 0.0f),
					Math::vec4(baseCrossDirection.y * baseCrossDirection.y, (-baseCrossDirection.x) * (-baseCrossDirection.x), 0.0f, 0.0f),
					Math::vec4(0.0f, 0.0f, 0.0f, 0.0f));
				Math::mat4 capsuleRotationMatrix = Math::mat4() + skewSymmetricCrossProductMatrix + (squaredSkewSymmetricCrossProductMatrix * (1.0f / (1.0f + cosAngle)));
				capsuleRotation = Math::eulerAnglesToQuat(Math::rotationMatrixToEulerAngles(capsuleRotationMatrix));
			}
			bodySettings.mPosition = JPH::Vec3(capsuleCenter.x, capsuleCenter.y, capsuleCenter.z);
			bodySettings.mRotation = JPH::Quat(capsuleRotation.b, capsuleRotation.c, capsuleRotation.d, capsuleRotation.a);
		}

		m_entityToBodyID[entity] = bodyInterface.CreateAndAddBody(bodySettings, rigidbody.isStatic ? JPH::EActivation::DontActivate : JPH::EActivation::Activate);
	}
}

void NtshEngn::PhysicsModule::onEntityComponentRemoved(Entity entity, Component componentID) {
	NTSHENGN_UNUSED(componentID);

	JPH::BodyInterface& bodyInterface = m_physicsSystem->GetBodyInterface();

	if (ecs->hasComponent<Rigidbody>(entity) && ecs->hasComponent<Collidable>(entity)) {
		bodyInterface.RemoveBody(m_entityToBodyID[entity]);
		bodyInterface.DestroyBody(m_entityToBodyID[entity]);
		m_entityToBodyID.erase(entity);
	}
}

NtshEngn::Math::vec3 NtshEngn::PhysicsModule::getCenter(const ColliderShape* shape) {
	if (shape->getType() == ColliderShapeType::Box) {
		return getCenter(static_cast<const ColliderBox*>(shape));
	}
	else if (shape->getType() == ColliderShapeType::Sphere) {
		return getCenter(static_cast<const ColliderSphere*>(shape));
	}
	else if (shape->getType() == ColliderShapeType::Capsule) {
		return getCenter(static_cast<const ColliderCapsule*>(shape));
	}

	return Math::vec3(0.0f, 0.0f, 0.0f);
}

NtshEngn::Math::vec3 NtshEngn::PhysicsModule::getCenter(const ColliderBox* box) {
	return box->center;
}

NtshEngn::Math::vec3 NtshEngn::PhysicsModule::getCenter(const ColliderSphere* sphere) {
	return sphere->center;
}

NtshEngn::Math::vec3 NtshEngn::PhysicsModule::getCenter(const ColliderCapsule* capsule) {
	return (capsule->base + capsule->tip) / 2.0f;
}

void NtshEngn::PhysicsModule::transform(ColliderShape* shape, const Math::vec3& translation, const Math::vec3& rotation, const Math::vec3& scale) {
	if (shape->getType() == ColliderShapeType::Box) {
		transform(static_cast<ColliderBox*>(shape), translation, rotation, scale);
	}
	else if (shape->getType() == ColliderShapeType::Sphere) {
		transform(static_cast<ColliderSphere*>(shape), translation, rotation, scale);
	}
	else if (shape->getType() == ColliderShapeType::Capsule) {
		transform(static_cast<ColliderCapsule*>(shape), translation, rotation, scale);
	}
}

void NtshEngn::PhysicsModule::transform(ColliderBox* box, const Math::vec3& translation, const Math::vec3& rotation, const Math::vec3& scale) {
	box->halfExtent.x *= std::abs(scale.x);
	box->halfExtent.y *= std::abs(scale.y);
	box->halfExtent.z *= std::abs(scale.z);

	const Math::quat originalRotation = Math::eulerAnglesToQuat(box->rotation);
	const Math::quat modelRotation = Math::eulerAnglesToQuat(rotation);
	box->rotation = Math::quatToEulerAngles(originalRotation * modelRotation);
	const Math::mat4 rotationMatrix = Math::translate(translation) * Math::rotate(box->rotation.x, Math::vec3(1.0f, 0.0f, 0.0f)) *
		Math::rotate(box->rotation.y, Math::vec3(0.0f, 1.0f, 0.0f)) *
		Math::rotate(box->rotation.z, Math::vec3(0.0f, 0.0f, 1.0f)) * Math::translate(-translation);
	box->center = Math::vec3(rotationMatrix * Math::vec4(box->center + translation, 1.0f));
}

void NtshEngn::PhysicsModule::transform(ColliderSphere* sphere, const Math::vec3& translation, const Math::vec3& rotation, const Math::vec3& scale) {
	const Math::mat4 rotationMatrix = Math::translate(translation) * Math::rotate(rotation.x, Math::vec3(1.0f, 0.0f, 0.0f)) *
		Math::rotate(rotation.y, Math::vec3(0.0f, 1.0f, 0.0f)) *
		Math::rotate(rotation.z, Math::vec3(0.0f, 0.0f, 1.0f)) * Math::translate(-translation);

	sphere->center = Math::vec3(rotationMatrix * Math::vec4(sphere->center + translation, 1.0f));
	sphere->radius *= std::max(std::abs(scale.x), std::max(std::abs(scale.y), std::abs(scale.z)));
}

void NtshEngn::PhysicsModule::transform(ColliderCapsule* capsule, const Math::vec3& translation, const Math::vec3& rotation, const Math::vec3& scale) {
	capsule->base += translation;
	capsule->tip += translation;

	const Math::mat4 rotationMatrix = Math::rotate(rotation.x, Math::vec3(1.0f, 0.0f, 0.0f)) *
		Math::rotate(rotation.y, Math::vec3(0.0f, 1.0f, 0.0f)) *
		Math::rotate(rotation.z, Math::vec3(0.0f, 0.0f, 1.0f));

	capsule->base = Math::vec3(rotationMatrix * Math::vec4(capsule->base, 1.0f));
	capsule->tip = Math::vec3(rotationMatrix * Math::vec4(capsule->tip, 1.0f));

	capsule->radius *= std::max(std::abs(scale.x), std::max(std::abs(scale.y), std::abs(scale.z)));
}

extern "C" NTSHENGN_MODULE_API NtshEngn::PhysicsModuleInterface* createModule() {
	return new NtshEngn::PhysicsModule;
}

extern "C" NTSHENGN_MODULE_API void destroyModule(NtshEngn::PhysicsModuleInterface* m) {
	delete m;
}