#pragma once

#include "CoreMinimal.h"
#include "IDetailCustomization.h"

class IDetailLayoutBuilder;

/**
 * Custom details panel for UJoltSettings. Replaces the default array widget for the ObjectLayers
 * property with a symmetric collision matrix: rows and columns are object-layer names, each cell
 * is a checkbox that toggles CollidesWith on both layers at once.
 */
class FJoltSettingsDetails : public IDetailCustomization
{
public:
	static TSharedRef<IDetailCustomization> MakeInstance();

	virtual void CustomizeDetails(IDetailLayoutBuilder& DetailBuilder) override;

private:
	TWeakObjectPtr<class UJoltSettings> WeakSettings;
	// Indirect handle whose lifetime matches this customizer instance. The lambda captures a
	// TWeakPtr so it becomes a no-op once this customizer is destroyed during ForceRefreshDetails.
	TSharedPtr<IDetailLayoutBuilder*> BuilderHandle;

	// Toggle collision between two object layers. Mirrors the change on both sides so the relation
	// stays symmetric. Records a transaction so undo works.
	void ToggleCollision(FName LayerA, FName LayerB);

	// Whether these two layers currently collide according to the UJoltSettings CDO. Checks the
	// relation in both directions and treats an OR as the source of truth so a half-edited file
	// still renders cleanly.
	ECheckBoxState GetCollisionState(FName LayerA, FName LayerB) const;
};
