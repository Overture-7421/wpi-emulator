#include "client/drive_overlays.h"

#include <QFontDatabase>
#include <QHBoxLayout>
#include <QLabel>
#include <QProgressBar>
#include <QPushButton>
#include <QStringList>
#include <QVBoxLayout>

#include "client/client_text.h"
#include "ui/qt_text.h"

namespace wpiemu::app::client {
namespace {

constexpr ui::Tr tr{"Client"};

QString toneColor(runview::Tone tone) {
    switch (tone) {
        case runview::Tone::Error: return QStringLiteral("#e5484d");
        case runview::Tone::Warning: return QStringLiteral("#e0a526");
        case runview::Tone::Success: return QStringLiteral("#2fb36d");
        case runview::Tone::Info: return QStringLiteral("#4a9eff");
        case runview::Tone::Neutral: break;
    }
    return QStringLiteral("#9aa4b2");
}

const char* kCardStyle =
    "QFrame#%1 { background: rgba(20,24,32,215); border: 1px solid rgba(255,255,255,60); border-radius: 8px; }"
    "QFrame#%1 QLabel { color: white; background: transparent; }";

}  // namespace

ClientBanner::ClientBanner(QWidget* parent) : QFrame(parent) {
    setObjectName(QStringLiteral("client.banner"));
    setStyleSheet(QString::fromLatin1(kCardStyle).arg(objectName()));
    title_ = new QLabel(this);
    title_->setObjectName(QStringLiteral("client.bannerTitle"));
    QFont f = title_->font();
    f.setBold(true);
    f.setPointSizeF(f.pointSizeF() * 1.25);
    title_->setFont(f);
    title_->setAlignment(Qt::AlignCenter);
    title_->setWordWrap(true);
    detail_ = new QLabel(this);
    detail_->setObjectName(QStringLiteral("client.bannerDetail"));
    detail_->setAlignment(Qt::AlignCenter);
    detail_->setWordWrap(true);
    busy_ = new QProgressBar(this);
    busy_->setRange(0, 0);
    busy_->setTextVisible(false);
    busy_->setMaximumHeight(5);
    reconnect_ = new QPushButton(tr("Reconnect"), this);
    reconnect_->setObjectName(QStringLiteral("client.reconnect"));
    leave_ = new QPushButton(tr("Back to the join screen"), this);
    leave_->setObjectName(QStringLiteral("client.bannerLeave"));
    auto* buttons = new QHBoxLayout;
    buttons->addStretch(1);
    buttons->addWidget(reconnect_);
    buttons->addWidget(leave_);
    buttons->addStretch(1);
    auto* layout = new QVBoxLayout(this);
    layout->setContentsMargins(18, 12, 18, 12);
    layout->setSpacing(6);
    layout->addWidget(title_);
    layout->addWidget(detail_);
    layout->addWidget(busy_);
    layout->addLayout(buttons);
    setMinimumWidth(380);
    setMaximumWidth(560);
    connect(reconnect_, &QPushButton::clicked, this, &ClientBanner::reconnectRequested);
    connect(leave_, &QPushButton::clicked, this, &ClientBanner::leaveRequested);
    hide();
}

void ClientBanner::setPresentation(const Presentation& p) {
    if (p.banner == Banner::None || p.screen != Screen::Drive) {
        hide();
        return;
    }
    title_->setText(bannerTitle(p.banner));
    title_->setStyleSheet(QStringLiteral("color: %1;").arg(toneColor(p.tone)));
    const QString detail = bannerDetail(p);
    detail_->setText(detail);
    detail_->setVisible(!detail.isEmpty());
    busy_->setVisible(p.busy);
    reconnect_->setVisible(p.canReconnect);
    // While playing (lobby, paused, stalled) the banner is only a notice; a way out is offered once the link is gone.
    const bool ended = p.tone == runview::Tone::Error;
    leave_->setVisible(ended || p.banner == Banner::Reconnecting);
    leave_->setText(ended ? tr("Back to the join screen") : tr("Cancel"));
    adjustSize();
    show();
    raise();
}

QStringList diagnosticsLines(const core::match::MatchClientView& view) {
    const core::match::ClientDiagnostics& d = view.diagnostics;
    const auto ms = [](double v) { return QStringLiteral("%1 ms").arg(v, 0, 'f', 1); };
    QStringList lines;
    lines << QStringLiteral("%1: %2").arg(tr("State"), stateWord(view.connection.state));
    lines << QStringLiteral("%1: %2 (%3 %4)").arg(tr("Ping"), ms(d.rttMs), tr("min"), ms(d.rttMinMs));
    lines << QStringLiteral("%1: %2").arg(tr("Jitter"), ms(d.jitterMs));
    lines << QStringLiteral("%1: %2 ‰ (%3 Hz)").arg(tr("Packet loss"), QString::number(d.lossPermille, 'f', 1), QString::number(d.snapshotRateHz, 'f', 0));
    lines << QStringLiteral("%1: %2").arg(tr("Snapshot age"), ms(d.snapshotAgeMs));
    lines << QStringLiteral("%1: %2").arg(tr("Buffer delay"), ms(d.bufferDelayMs));
    lines << QStringLiteral("%1: %2 cm (p95 %3 cm, %4°)")
                 .arg(tr("Prediction error"), QString::number(d.predictionErrorM * 100.0, 'f', 1),
                      QString::number(d.predictionErrorP95M * 100.0, 'f', 1),
                      QString::number(d.predictionErrorYawRad * 57.29577951308232, 'f', 1));
    lines << QStringLiteral("%1: %2 · %3 %4").arg(tr("Prediction ahead"), ms(d.predictionAheadMs), tr("lead"), QString::number(d.inputLeadTicks) + QStringLiteral(" ticks"));
    lines << QStringLiteral("%1: %2 · %3 %4")
                 .arg(tr("Corrections"), QString::number(d.reconciliations), tr("jumps"), QString::number(d.snaps));
    return lines;
}

DiagnosticsOverlay::DiagnosticsOverlay(QWidget* parent) : QFrame(parent) {
    setObjectName(QStringLiteral("client.diagnostics"));
    setStyleSheet(QString::fromLatin1(kCardStyle).arg(objectName()));
    text_ = new QLabel(this);
    text_->setObjectName(QStringLiteral("client.diagnosticsText"));
    text_->setFont(QFontDatabase::systemFont(QFontDatabase::FixedFont));
    auto* layout = new QVBoxLayout(this);
    layout->setContentsMargins(10, 8, 10, 8);
    layout->addWidget(text_);
    hide();
}

void DiagnosticsOverlay::setView(const core::match::MatchClientView* view) {
    text_->setText(view != nullptr ? diagnosticsLines(*view).join(QLatin1Char('\n')) : QString());
    adjustSize();
}

}  // namespace wpiemu::app::client
