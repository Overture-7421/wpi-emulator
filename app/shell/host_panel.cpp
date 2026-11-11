#include "shell/host_panel.h"

#include <QFormLayout>
#include <QHBoxLayout>
#include <QHeaderView>
#include <QSignalBlocker>
#include <QVBoxLayout>

#include "ui/widget_support.h"

namespace wpiemu::app {

namespace {

enum Column { PlayerColumn, StationColumn, PingColumn, LossColumn, KickColumn, ColumnCount };

}  // namespace

HostPanel::HostPanel(HostPanelModel& model, const UiResources& ui, QWidget* parent) : QWidget(parent), model_(model) {
    setObjectName(QStringLiteral("HostPanel"));
    auto* layout = new QVBoxLayout(this);
    layout->setContentsMargins(12, 12, 12, 12);
    layout->setSpacing(8);

    layout->addWidget(ui::headingLabel(tr("Practice match"), this));
    layout->addWidget(ui::mutedLabel(tr("Let other players join your simulation. Your robot code runs here; they drive their own robots."), this));

    mode_ = new QComboBox(this);
    mode_->addItem(tr("Off"), static_cast<int>(host::HostingMode::Off));
    mode_->addItem(tr("Direct (LAN or port forwarding)"), static_cast<int>(host::HostingMode::Direct));
    mode_->addItem(tr("Relay (over the internet)"), static_cast<int>(host::HostingMode::Relay));
    port_ = new QSpinBox(this);
    port_->setRange(0, 65535);
    port_->setSpecialValueText(tr("Any free port"));
    players_ = new QSpinBox(this);
    players_->setRange(static_cast<int>(host::kMinHostingPlayers), static_cast<int>(sdk::match::kMaxRobotsPerMatch));
    players_->setToolTip(tr("Everyone in the match, including your own robot."));
    portLabel_ = new QLabel(tr("UDP port"), this);

    auto* form = new QFormLayout;
    form->setContentsMargins(0, 0, 0, 0);
    form->addRow(tr("Hosting"), mode_);
    form->addRow(portLabel_, port_);
    form->addRow(tr("Players"), players_);
    layout->addLayout(form);

    relayInfo_ = ui::captionLabel({}, this);
    relayInfo_->setWordWrap(true);
    relayInfo_->setTextInteractionFlags(Qt::TextSelectableByMouse);
    layout->addWidget(relayInfo_);

    advancedToggle_ = new QToolButton(this);
    advancedToggle_->setText(tr("Advanced"));
    advancedToggle_->setCheckable(true);
    advancedToggle_->setAutoRaise(true);
    advancedToggle_->setToolButtonStyle(Qt::ToolButtonTextBesideIcon);
    advancedToggle_->setArrowType(Qt::RightArrow);
    layout->addWidget(advancedToggle_, 0, Qt::AlignLeft);

    advanced_ = new QWidget(this);
    auto* advancedForm = new QFormLayout(advanced_);
    advancedForm->setContentsMargins(12, 0, 0, 0);
    relayLabel_ = new QLabel(tr("Relay"), advanced_);
    relay_ = new QComboBox(advanced_);
    relay_->addItem(tr("Production (default)"), static_cast<int>(RelayChoice::Production));
    relay_->addItem(tr("Development (testing)"), static_cast<int>(RelayChoice::Development));
    relay_->addItem(tr("Custom address"), static_cast<int>(RelayChoice::Custom));
    customRelayLabel_ = new QLabel(tr("Relay address"), advanced_);
    customRelay_ = new QLineEdit(advanced_);
    customRelay_->setPlaceholderText(tr("host:port"));
    bindLabel_ = new QLabel(tr("Listen on"), advanced_);
    bind_ = new QLineEdit(advanced_);
    bind_->setToolTip(tr("Direct mode: the local address to listen on. \"::\" is every network interface."));
    advancedForm->addRow(relayLabel_, relay_);
    advancedForm->addRow(customRelayLabel_, customRelay_);
    advancedForm->addRow(bindLabel_, bind_);
    advanced_->setVisible(false);
    layout->addWidget(advanced_);

    problem_ = new Banner(ui, this);
    hint_ = new Banner(ui, this);
    phase_ = new Banner(ui, this);
    layout->addWidget(problem_);
    layout->addWidget(hint_);
    layout->addWidget(phase_);

    inviteRow_ = new QWidget(this);
    auto* inviteLayout = new QHBoxLayout(inviteRow_);
    inviteLayout->setContentsMargins(0, 0, 0, 0);
    auto* inviteCaption = new QLabel(tr("Invite"), inviteRow_);
    invite_ = new QLineEdit(inviteRow_);
    invite_->setReadOnly(true);
    invite_->setToolTip(tr("Share this with the players. Anyone who has it can join."));
    copy_ = new QPushButton(tr("Copy"), inviteRow_);
    copy_->setToolTip(tr("Copy the invite to the clipboard"));
    inviteLayout->addWidget(inviteCaption);
    inviteLayout->addWidget(invite_, 1);
    inviteLayout->addWidget(copy_);
    layout->addWidget(inviteRow_);
    endpoint_ = ui::captionLabel({}, this);
    endpoint_->setTextInteractionFlags(Qt::TextSelectableByMouse);
    layout->addWidget(endpoint_);

    layout->addWidget(ui::headingLabel(tr("Players"), this));
    roster_ = new QTableWidget(0, ColumnCount, this);
    roster_->setHorizontalHeaderLabels({tr("Player"), tr("Station"), tr("Ping"), tr("Loss"), QString()});
    roster_->verticalHeader()->setVisible(false);
    roster_->setEditTriggers(QAbstractItemView::NoEditTriggers);
    roster_->setSelectionMode(QAbstractItemView::NoSelection);
    roster_->setFocusPolicy(Qt::NoFocus);
    roster_->horizontalHeader()->setSectionResizeMode(PlayerColumn, QHeaderView::Stretch);
    roster_->horizontalHeader()->setSectionResizeMode(StationColumn, QHeaderView::ResizeToContents);
    roster_->horizontalHeader()->setSectionResizeMode(PingColumn, QHeaderView::ResizeToContents);
    roster_->horizontalHeader()->setSectionResizeMode(LossColumn, QHeaderView::ResizeToContents);
    roster_->horizontalHeader()->setSectionResizeMode(KickColumn, QHeaderView::ResizeToContents);
    roster_->setMinimumHeight(160);
    layout->addWidget(roster_, 1);

    connect(mode_, &QComboBox::activated, this, [this](int i) { model_.setMode(static_cast<host::HostingMode>(mode_->itemData(i).toInt())); });
    connect(port_, &QSpinBox::editingFinished, this, [this] { model_.setPort(port_->value()); });
    connect(players_, &QSpinBox::editingFinished, this, [this] { model_.setMaxPlayers(players_->value()); });
    connect(advancedToggle_, &QToolButton::toggled, this, [this](bool open) {
        advanced_->setVisible(open);
        advancedToggle_->setArrowType(open ? Qt::DownArrow : Qt::RightArrow);
    });
    connect(relay_, &QComboBox::activated, this, [this](int i) { model_.setRelayChoice(static_cast<RelayChoice>(relay_->itemData(i).toInt())); });
    connect(customRelay_, &QLineEdit::editingFinished, this, [this] { model_.setCustomRelay(customRelay_->text()); });
    connect(bind_, &QLineEdit::editingFinished, this, [this] { model_.setBindHost(bind_->text()); });
    connect(copy_, &QPushButton::clicked, this, [this] { model_.copyInvite(); });
    connect(&model_, &HostPanelModel::changed, this, &HostPanel::refresh);

    if (model_.relayChoice() != RelayChoice::Production) advancedToggle_->setChecked(true);
    refresh();
}

QPushButton* HostPanel::kickButton(int row) const {
    return qobject_cast<QPushButton*>(roster_->cellWidget(row, KickColumn));
}

void HostPanel::refresh() {
    const QSignalBlocker modeBlock(mode_), portBlock(port_), playersBlock(players_), relayBlock(relay_), customBlock(customRelay_),
        bindBlock(bind_);
    const host::HostingMode mode = model_.mode();
    mode_->setCurrentIndex(mode_->findData(static_cast<int>(mode)));
    port_->setValue(model_.port());
    players_->setValue(model_.maxPlayers());
    relay_->setCurrentIndex(relay_->findData(static_cast<int>(model_.relayChoice())));
    if (!customRelay_->hasFocus()) customRelay_->setText(model_.customRelay());
    if (!bind_->hasFocus()) bind_->setText(model_.bindHost());

    const bool direct = mode == host::HostingMode::Direct;
    const bool relayMode = mode == host::HostingMode::Relay;
    const bool custom = model_.relayChoice() == RelayChoice::Custom;
    port_->setVisible(direct);
    portLabel_->setVisible(direct);
    bindLabel_->setVisible(direct);
    bind_->setVisible(direct);
    relayLabel_->setVisible(relayMode);
    relay_->setVisible(relayMode);
    customRelayLabel_->setVisible(relayMode && custom);
    customRelay_->setVisible(relayMode && custom);
    advancedToggle_->setVisible(mode != host::HostingMode::Off);
    if (mode == host::HostingMode::Off) advanced_->setVisible(false);
    else advanced_->setVisible(advancedToggle_->isChecked());

    const QString relayEndpoint = model_.relayEndpoint();
    relayInfo_->setVisible(relayMode);
    relayInfo_->setText(relayEndpoint.isEmpty() ? tr("Relay: not configured for this build") : tr("Relay: %1").arg(relayEndpoint));

    if (model_.problem().isEmpty()) problem_->clear();
    else problem_->setMessage(BannerKind::Error, tr("Not saved: %1").arg(model_.problem()));

    const QString hint = model_.launchHint();
    if (hint.isEmpty()) hint_->clear();
    else hint_->setMessage(BannerKind::Info, hint);

    const HostPhaseBanner phase = model_.phase();
    if (!phase.visible) phase_->clear();
    else phase_->setMessage(phase.kind, tr("%1: %2").arg(phase.title, phase.detail));

    const bool hasInvite = model_.canCopyInvite();
    inviteRow_->setVisible(hasInvite);
    invite_->setText(model_.invite());
    copy_->setEnabled(hasInvite);
    const QString endpoint = model_.directEndpoint();
    endpoint_->setVisible(!endpoint.isEmpty());
    endpoint_->setText(tr("Listening on %1").arg(endpoint));

    const QList<HostRosterRow> rows = model_.roster();
    roster_->setRowCount(static_cast<int>(rows.size()));
    for (int i = 0; i < rows.size(); ++i) {
        const HostRosterRow& r = rows[i];
        auto cell = [&](Column column, const QString& text) {
            auto* item = new QTableWidgetItem(text);
            roster_->setItem(i, column, item);
            return item;
        };
        cell(PlayerColumn, r.isHost ? tr("%1 (you)").arg(r.name) : r.name);
        cell(StationColumn, r.station)->setForeground(r.stationColor);
        cell(PingColumn, r.ping);
        cell(LossColumn, r.loss);
        roster_->removeCellWidget(i, KickColumn);
        if (r.canKick) {
            auto* kick = new QPushButton(tr("Kick"), roster_);
            kick->setToolTip(tr("Remove %1 from the match").arg(r.name));
            const sdk::match::ParticipantId id = r.id;
            connect(kick, &QPushButton::clicked, this, [this, id] { model_.kick(id); });
            roster_->setCellWidget(i, KickColumn, kick);
        }
    }
}

}  // namespace wpiemu::app
