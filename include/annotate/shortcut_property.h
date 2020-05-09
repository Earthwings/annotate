#pragma once

#include <rviz/properties/string_property.h>
#include <QShortcut>
#include <rviz/display.h>

class ShortcutProperty : public rviz::StringProperty
{
  Q_OBJECT
public:
  ShortcutProperty(const QString& name = QString(), const QString& default_value = QString(),
                   const QString& description = QString(), rviz::Property* parent = 0, const char* changed_slot = 0,
                   QObject* receiver = 0);

  /**
   * Install a shortcut in target that calls trigger_slot in receiver. The shortcut is derived from getName(),
   * which should be a shortcut string that Qt understands (cf QKeySequence). If a shortcut is invalid or used
   * already, a warning status is set in display.
   */
  void createShortcut(rviz::Display* display, QWidget* target, QObject* receiver, const char* trigger_slot);
  void setEnabled(bool enabled);

private Q_SLOTS:
  void updateShortcut();
  void handleAmbiguousShortcut();

private:
  QString statusName() const;
  QShortcut* shortcut_{ nullptr };
  rviz::Display* display_{ nullptr };
};
