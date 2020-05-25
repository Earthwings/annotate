#include <annotate/file_dialog_property.h>
#include <rviz/properties/property.h>
#include <QFileDialog>
#include <QPushButton>

namespace annotate
{
FileDialogProperty::FileDialogProperty(const QString& name, const QString& default_value, const QString& description,
                                       rviz::Property* parent, const char* changed_slot, QObject* receiver)
  : rviz::Property(name, default_value, description, parent, changed_slot, receiver)
{
  // does nothing
}

QWidget* FileDialogProperty::createEditor(QWidget* parent, const QStyleOptionViewItem& option)
{
  FileDialogEditor* editor = new FileDialogEditor(this, parent, mode_);
  editor->setFrame(false);
  return editor;
}

void FileDialogProperty::setMode(Mode mode)
{
  mode_ = mode;
}

FileDialogEditor::FileDialogEditor(FileDialogProperty* property, QWidget* parent, FileDialogProperty::Mode mode)
  : rviz::LineEditWithButton(parent), property_(property), mode_(mode)
{
  // does nothing
}

void FileDialogEditor::onButtonClick()
{
  auto property = property_;  // same hack as in ColorEditor::onButtonClick()
  QString file;
  switch (mode_)
  {
    case FileDialogProperty::ExistingDirectory:
      file = QFileDialog::getExistingDirectory(parentWidget(), "Open Existing Directory", QString());
      break;
    case FileDialogProperty::OpenFileName:
      file = QFileDialog::getOpenFileName(parentWidget(), "Open Existing Annotation File", QString(),
                                          "Annotation Files (*.yaml *.yml *.annotate)");
      break;
    case FileDialogProperty::SaveFileName:
      file = QFileDialog::getSaveFileName(parentWidget(), "Save Annotation File", QString(),
                                          "Annotation Files (*.yaml *.yml *.annotate)");
      break;
  }
  if (!file.isEmpty())
  {
    property->setValue(file);
  }
}

}  // namespace annotate
