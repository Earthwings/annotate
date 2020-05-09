#include <rviz/properties/property.h>
#include <annotate/shortcut_property.h>
#include <QFileDialog>
#include <QPushButton>

ShortcutProperty::ShortcutProperty(const QString& name, const QString& default_value, const QString& description,
                                   rviz::Property* parent, const char* changed_slot, QObject* receiver)
  : rviz::StringProperty(name, default_value, description, parent, changed_slot, receiver)
{
  connect(this, SIGNAL(changed()), this, SLOT(updateShortcut()));
}

void ShortcutProperty::createShortcut(rviz::Display* display, QWidget* target, QObject* receiver,
                                      const char* trigger_slot)
{
  display_ = display;
  shortcut_ = new QShortcut(target);
  connect(shortcut_, SIGNAL(activated()), receiver, trigger_slot);
  connect(shortcut_, SIGNAL(activatedAmbiguously()), this, SLOT(handleAmbiguousShortcut()));
  updateShortcut();
}

QString ShortcutProperty::statusName() const
{
  return QString("Shortcut %1").arg(getName());
}

void ShortcutProperty::setEnabled(bool enabled)
{
  if (shortcut_)
  {
    shortcut_->setEnabled(enabled);
  }
}

void ShortcutProperty::updateShortcut()
{
  if (shortcut_)
  {
    auto const key_sequence = QKeySequence(getString());
    if (key_sequence.isEmpty())
    {
      if (display_)
      {
        display_->setStatus(rviz::StatusProperty::Warn, statusName(),
                            QString("The keyboard shortcut '%1' is invalid. Please choose a valid shortcut like "
                                    "'Ctrl+D'.")
                                .arg(getString()));
      }
    }
    else
    {
      shortcut_->setKey(key_sequence);
      if (display_)
      {
        display_->deleteStatus(statusName());
      }
    }
  }
}

void ShortcutProperty::handleAmbiguousShortcut()
{
  if (display_)
  {
    display_->setStatus(
        rviz::StatusProperty::Warn, statusName(),
        QString("The keyboard shortcut '%1' is already used. Please choose a different one.").arg(getString()));
  }
}
