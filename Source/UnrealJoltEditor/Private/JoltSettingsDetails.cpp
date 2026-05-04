#include "JoltSettingsDetails.h"
#include "DetailCategoryBuilder.h"
#include "DetailLayoutBuilder.h"
#include "DetailWidgetRow.h"
#include "IDetailChildrenBuilder.h"
#include "JoltSettings.h"
#include "ScopedTransaction.h"
#include "Widgets/Input/SCheckBox.h"
#include "Widgets/Layout/SGridPanel.h"
#include "Widgets/Layout/SScrollBox.h"
#include "Widgets/SBoxPanel.h"
#include "Widgets/Text/STextBlock.h"

#define LOCTEXT_NAMESPACE "JoltSettingsDetails"

TSharedRef<IDetailCustomization> FJoltSettingsDetails::MakeInstance()
{
	return MakeShared<FJoltSettingsDetails>();
}

void FJoltSettingsDetails::CustomizeDetails(IDetailLayoutBuilder& DetailBuilder)
{
	TArray<TWeakObjectPtr<UObject>> ObjectsCustomized;
	DetailBuilder.GetObjectsBeingCustomized(ObjectsCustomized);
	if (ObjectsCustomized.Num() == 0)
	{
		return;
	}

	UJoltSettings* Settings = Cast<UJoltSettings>(ObjectsCustomized[0].Get());
	if (Settings == nullptr)
	{
		return;
	}
	WeakSettings = Settings;

	// BuilderHandle lifetime == this customizer's lifetime. The lambda captures a TWeakPtr so it
	// silently no-ops after ForceRefreshDetails destroys and replaces this customizer instance.
	BuilderHandle = MakeShared<IDetailLayoutBuilder*>(&DetailBuilder);
	TWeakPtr<IDetailLayoutBuilder*> WeakHandle = BuilderHandle;

	// Rebuild the matrix whenever layers are added, removed, or renamed.
	FCoreUObjectDelegates::OnObjectPropertyChanged.AddWeakLambda(Settings,
		[WeakHandle, WeakObj = TWeakObjectPtr<UJoltSettings>(Settings)](
			UObject* Object, FPropertyChangedEvent& Event)
		{
			if (Object != WeakObj.Get()) return;
			// Skip per-keystroke interactive events (e.g. typing a layer name character by character).
			if (Event.ChangeType == EPropertyChangeType::Interactive) return;
			const FName Member = Event.GetMemberPropertyName();
			if (Member != GET_MEMBER_NAME_CHECKED(UJoltSettings, ObjectLayers) &&
				Member != GET_MEMBER_NAME_CHECKED(UJoltSettings, BroadphaseLayers))
				return;
			if (TSharedPtr<IDetailLayoutBuilder*> Handle = WeakHandle.Pin())
				(*Handle)->ForceRefreshDetails();
		});

	// Default property rendering for BroadphaseLayers and ObjectLayers is fine — users edit the
	// list of rows through the usual array widget. The matrix widget lives just below, in a new
	// "Collision Matrix" category, so the two views stay side by side.
	IDetailCategoryBuilder& MatrixCategory = DetailBuilder.EditCategory(
		TEXT("Layers"),
		LOCTEXT("LayersCategory", "Layers"),
		ECategoryPriority::Important);

	// Build the NxN grid from the current settings snapshot. Rebuilt on every details refresh.
	TSharedRef<SGridPanel> Grid = SNew(SGridPanel);

	const int32 LayerCount = Settings->ObjectLayers.Num();

	// Header row: one empty top-left cell, then one column header per layer.
	Grid->AddSlot(0, 0)
		.Padding(4)
		[
			SNew(STextBlock)
			.Text(LOCTEXT("MatrixCorner", ""))
		];

	for (int32 Col = 0; Col < LayerCount; ++Col)
	{
		const FName ColName = Settings->ObjectLayers[Col].Name;
		Grid->AddSlot(Col + 1, 0)
			.Padding(4)
			.HAlign(HAlign_Center)
			[
				SNew(STextBlock)
				.Text(FText::FromName(ColName))
			];
	}

	// One row per layer. First cell is the row header, rest are checkboxes.
	for (int32 Row = 0; Row < LayerCount; ++Row)
	{
		const FName RowName = Settings->ObjectLayers[Row].Name;

		Grid->AddSlot(0, Row + 1)
			.Padding(4)
			.VAlign(VAlign_Center)
			[
				SNew(STextBlock)
				.Text(FText::FromName(RowName))
			];

		for (int32 Col = 0; Col < LayerCount; ++Col)
		{
			const FName ColName = Settings->ObjectLayers[Col].Name;

			TSharedRef<SCheckBox> Checkbox = SNew(SCheckBox)
				.IsChecked_Lambda([this, RowName, ColName]() {
					return GetCollisionState(RowName, ColName);
				})
				.OnCheckStateChanged_Lambda([this, RowName, ColName](ECheckBoxState NewState) {
					ToggleCollision(RowName, ColName);
				});

			Grid->AddSlot(Col + 1, Row + 1)
				.Padding(4)
				.HAlign(HAlign_Center)
				.VAlign(VAlign_Center)
				[
					Checkbox
				];
		}
	}

	MatrixCategory.AddCustomRow(LOCTEXT("CollisionMatrixRow", "Collision Matrix"))
		.WholeRowContent()
		[
			SNew(SVerticalBox)
			+ SVerticalBox::Slot()
			.AutoHeight()
			.Padding(4)
			[
				SNew(STextBlock)
				.Text(LOCTEXT("CollisionMatrixTitle", "Collision Matrix (symmetric)"))
				.Font(IDetailLayoutBuilder::GetDetailFontBold())
			]
			+ SVerticalBox::Slot()
			.AutoHeight()
			[
				SNew(SScrollBox)
				.Orientation(Orient_Horizontal)
				+ SScrollBox::Slot()
				[
					Grid
				]
			]
		];
}

ECheckBoxState FJoltSettingsDetails::GetCollisionState(FName LayerA, FName LayerB) const
{
	UJoltSettings* Settings = WeakSettings.Get();
	if (Settings == nullptr)
	{
		return ECheckBoxState::Unchecked;
	}

	// Treat the relation as an OR of both directions so a half-edited file still renders the
	// checkbox consistently on both sides of the diagonal.
	bool bCollides = false;
	for (const FJoltObjectLayer& Layer : Settings->ObjectLayers)
	{
		if (Layer.Name == LayerA && Layer.CollidesWith.Contains(LayerB))
		{
			bCollides = true;
			break;
		}
		if (Layer.Name == LayerB && Layer.CollidesWith.Contains(LayerA))
		{
			bCollides = true;
			break;
		}
	}
	return bCollides ? ECheckBoxState::Checked : ECheckBoxState::Unchecked;
}

void FJoltSettingsDetails::ToggleCollision(FName LayerA, FName LayerB)
{
	UJoltSettings* Settings = WeakSettings.Get();
	if (Settings == nullptr)
	{
		return;
	}

	const FScopedTransaction Transaction(LOCTEXT("ToggleCollision", "Toggle Jolt layer collision"));
	Settings->Modify();

	const bool bCurrentlyCollides = GetCollisionState(LayerA, LayerB) == ECheckBoxState::Checked;

	// Apply the flip to both sides so the relation stays symmetric in one step. Going through
	// PostEditChangeProperty would also achieve this, but doing it inline here keeps the undo
	// transaction atomic and avoids a second settings save.
	for (FJoltObjectLayer& Layer : Settings->ObjectLayers)
	{
		if (Layer.Name == LayerA)
		{
			if (bCurrentlyCollides)
			{
				Layer.CollidesWith.Remove(LayerB);
			}
			else
			{
				Layer.CollidesWith.Add(LayerB);
			}
		}
		else if (Layer.Name == LayerB)
		{
			if (bCurrentlyCollides)
			{
				Layer.CollidesWith.Remove(LayerA);
			}
			else
			{
				Layer.CollidesWith.Add(LayerA);
			}
		}
	}
}

#undef LOCTEXT_NAMESPACE
