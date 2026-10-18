#pragma once

// One inspector section as a framed card: a coloured header band (accent bar, glyph tile, title,
// collapse arrow) over a body widget. The colour identifies the section kind everywhere, so Drive,
// Configure, Placement and Shapes are told apart at a glance in both light and dark themes.

#include <QFrame>
#include <QIcon>
#include <QString>
#include <QToolButton>

namespace wpiemu::app::editor {

enum class SectionRole { Drive, Configure, Placement, Shapes, Info };

class SectionCard : public QFrame {
    Q_OBJECT
public:
    SectionCard(const QString& title, SectionRole role, QWidget* body, bool collapsed = false, QWidget* parent = nullptr);

    static QColor colorFor(SectionRole role);
    static QString glyphFor(SectionRole role);
    static const char* nameFor(SectionRole role);

    SectionRole role() const { return role_; }
    QWidget* body() const { return body_; }
    QToolButton* header() const { return header_; }

private:
    SectionRole role_;
    QWidget* body_;
    QToolButton* header_;
};

}  // namespace wpiemu::app::editor
