#include <doctest/doctest.h>

#include <QApplication>
#include <QComboBox>
#include <QDate>
#include <QDateTime>
#include <QDoubleSpinBox>
#include <QLabel>
#include <QLineEdit>
#include <QScrollArea>
#include <QScrollBar>
#include <QSlider>
#include <QTest>
#include <QTime>
#include <QTranslator>
#include <QVBoxLayout>
#include <QWheelEvent>
#include <QWidget>
#include <QtTest/qtestspontaneevent.h>

#include <string_view>

#include "ui/qt_text.h"
#include "ui/widget_support.h"

namespace ui = wpiemu::app::ui;

namespace {

// Translates "Hello" in the "UiTest" context only.
class UiTestTranslator : public QTranslator {
public:
    UiTestTranslator() { QCoreApplication::installTranslator(this); }
    ~UiTestTranslator() override { QCoreApplication::removeTranslator(this); }

    bool isEmpty() const override { return false; }
    QString translate(const char* context, const char* sourceText, const char*, int) const override {
        return std::string_view(context) == "UiTest" && std::string_view(sourceText) == "Hello" ? QStringLiteral("Hola") : QString();
    }
};

qint64 localMs(const QDate& date, int h, int m, int s, int ms = 0) { return QDateTime(date, QTime(h, m, s, ms)).toMSecsSinceEpoch(); }

}  // namespace

TEST_CASE("qs decodes the SDK's UTF-8 strings and round-trips them") {
    CHECK(ui::qs("").isEmpty());
    CHECK(ui::qs("drive") == QStringLiteral("drive"));

    const std::string utf8 = "caf\xC3\xA9 \xC2\xB7 \xE2\x80\x94 \xE6\x97\xA5";
    const QString text = ui::qs(utf8);
    CHECK(text.size() == 10);
    CHECK(text[3] == QChar(0x00E9));
    CHECK(text[7] == QChar(0x2014));
    CHECK(text[9] == QChar(0x65E5));
    CHECK(text.toStdString() == utf8);
}

TEST_CASE("Tr translates in its own context and falls back to the source text") {
    constexpr ui::Tr tr{"UiTest"};
    constexpr ui::Tr other{"OtherContext"};
    CHECK(tr("Hello") == QStringLiteral("Hello"));   // nothing installed yet

    const UiTestTranslator translator;
    CHECK(tr("Hello") == QStringLiteral("Hola"));
    CHECK(tr("Hello") == QCoreApplication::translate("UiTest", "Hello"));
    CHECK(other("Hello") == QStringLiteral("Hello"));
    CHECK(tr("Goodbye") == QStringLiteral("Goodbye"));
}

TEST_CASE("clock text is the local time of day, zero padded, with optional milliseconds") {
    const QDate day(2024, 1, 15);
    CHECK(ui::clockText(localMs(day, 0, 0, 0)) == QStringLiteral("00:00:00"));
    CHECK(ui::clockText(localMs(day, 9, 5, 7, 999)) == QStringLiteral("09:05:07"));   // truncated, never rounded up
    CHECK(ui::clockText(localMs(day, 23, 59, 59, 999)) == QStringLiteral("23:59:59"));

    CHECK(ui::clockTextMs(localMs(day, 0, 0, 0)) == QStringLiteral("00:00:00.000"));
    CHECK(ui::clockTextMs(localMs(day, 9, 5, 7, 9)) == QStringLiteral("09:05:07.009"));
    CHECK(ui::clockTextMs(localMs(day, 23, 59, 59, 999)) == QStringLiteral("23:59:59.999"));

    const QDate beforeEpoch(1969, 12, 31);
    REQUIRE(localMs(beforeEpoch, 12, 34, 56) < 0);
    CHECK(ui::clockText(localMs(beforeEpoch, 12, 34, 56)) == QStringLiteral("12:34:56"));
}

TEST_CASE("muted and caption labels are disabled and only the muted one wraps") {
    QWidget parent;
    const QLabel plain(&parent);

    QLabel* muted = ui::mutedLabel(QStringLiteral("hint"), &parent);
    CHECK(muted->text() == QStringLiteral("hint"));
    CHECK(muted->parentWidget() == &parent);
    CHECK(muted->wordWrap());
    CHECK_FALSE(muted->isEnabled());

    QLabel* caption = ui::captionLabel(QStringLiteral("Position"), &parent);
    CHECK(caption->text() == QStringLiteral("Position"));
    CHECK_FALSE(caption->wordWrap());
    CHECK_FALSE(caption->isEnabled());
    CHECK(caption->font().pointSizeF() == doctest::Approx(plain.font().pointSizeF() * 0.85));
}

TEST_CASE("heading labels are upper-cased bold letter-spaced captions") {
    QWidget parent;
    const QLabel plain(&parent);
    QLabel* heading = ui::headingLabel(QStringLiteral("Drive motors"), &parent);
    CHECK(heading->text() == QStringLiteral("DRIVE MOTORS"));
    CHECK(heading->font().bold());
    CHECK(heading->font().letterSpacingType() == QFont::PercentageSpacing);
    CHECK(heading->font().letterSpacing() == doctest::Approx(108.0));
    CHECK(heading->font().pointSizeF() == doctest::Approx(plain.font().pointSizeF() * 0.8));
    CHECK_FALSE(heading->isEnabled());
}

TEST_CASE("quiet wheel: dropdowns and unfocused inputs let the wheel scroll the form instead") {
    QScrollArea area;
    auto* content = new QWidget;
    auto* layout = new QVBoxLayout(content);
    auto* holder = new QLineEdit(content);   // keeps focus, so a wheel notch giving focus to an input is detectable
    layout->addWidget(holder);
    auto* combo = ui::quietWheel(new QComboBox(content));
    combo->addItems({QStringLiteral("a"), QStringLiteral("b"), QStringLiteral("c")});
    auto* spin = ui::quietWheel(new QDoubleSpinBox(content));
    spin->setRange(0, 100);
    spin->setValue(50);
    auto* slider = ui::quietWheel(new QSlider(Qt::Horizontal, content));
    slider->setRange(0, 100);
    slider->setValue(50);
    layout->addWidget(combo);
    layout->addWidget(spin);
    layout->addWidget(slider);
    auto* filler = new QWidget(content);
    filler->setMinimumHeight(1500);
    layout->addWidget(filler);

    area.setWidget(content);
    area.setWidgetResizable(true);
    area.resize(300, 200);
    area.show();
    QApplication::processEvents();
    holder->setFocus();
    QApplication::processEvents();
    REQUIRE(area.verticalScrollBar()->maximum() > 0);

    const auto wheelOver = [&](QWidget* w) {
        QWheelEvent e(QPointF(w->width() / 2, w->height() / 2), w->mapToGlobal(QPointF(w->width() / 2, w->height() / 2)), QPoint(),
                      QPoint(0, -120), Qt::NoButton, Qt::NoModifier, Qt::NoScrollPhase, false);
        QSpontaneKeyEvent::setSpontaneous(&e);
        qApp->notify(w, &e);
    };

    for (QWidget* w : std::initializer_list<QWidget*>{combo, spin, slider}) {
        area.verticalScrollBar()->setValue(0);
        wheelOver(w);
        CHECK(area.verticalScrollBar()->value() > 0);
        CHECK_FALSE(w->hasFocus());
    }
    CHECK(combo->currentIndex() == 0);
    CHECK(spin->value() == doctest::Approx(50.0));
    CHECK(slider->value() == 50);

    spin->setFocus();
    QApplication::processEvents();
    REQUIRE(spin->hasFocus());
    wheelOver(spin);
    CHECK(spin->value() == doctest::Approx(49.0));
}

TEST_CASE("flashWidget styles a widget and clears the style after kFlashMs") {
    QWidget widget;
    ui::flashWidget(&widget, QStringLiteral("background: red;"));
    CHECK(widget.styleSheet() == QStringLiteral("background: red;"));
    QTest::qWait(ui::kFlashMs / 2);
    CHECK(widget.styleSheet() == QStringLiteral("background: red;"));
    QTest::qWait(ui::kFlashMs / 2 + 200);
    CHECK(widget.styleSheet().isEmpty());
}
