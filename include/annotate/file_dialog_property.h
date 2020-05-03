#pragma once

#include <rviz/properties/property.h>
#include "rviz/properties/line_edit_with_button.h"

class FileDialogProperty : public rviz::Property
{
  Q_OBJECT
public:
  enum Mode
  {
    ExistingDirectory,
    OpenFileName,
    SaveFileName
  };

  FileDialogProperty(const QString& name = QString(), const QString& default_value = QString(),
                     const QString& description = QString(), rviz::Property* parent = 0, const char* changed_slot = 0,
                     QObject* receiver = 0);

  void setMode(Mode mode);
  QWidget* createEditor(QWidget* parent, const QStyleOptionViewItem& option);

private:
  Mode mode_{ OpenFileName };
};

class FileDialogEditor : public rviz::LineEditWithButton
{
  Q_OBJECT
public:
  FileDialogEditor(FileDialogProperty* property, QWidget* parent, FileDialogProperty::Mode mode);
  void onButtonClick() override;

private:
  FileDialogProperty* property_;
  FileDialogProperty::Mode const mode_;
};
