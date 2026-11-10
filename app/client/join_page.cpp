#include "client/join_page.h"

#include <QCheckBox>
#include <QComboBox>
#include <QDir>
#include <QDoubleSpinBox>
#include <QFileDialog>
#include <QFileInfo>
#include <QFormLayout>
#include <QGroupBox>
#include <QHBoxLayout>
#include <QLabel>
#include <QLineEdit>
#include <QProgressBar>
#include <QPushButton>
#include <QToolButton>
#include <QVBoxLayout>

#include "client/client_text.h"
#include "ui/qt_text.h"
#include "ui/widget_support.h"

namespace wpiemu::app::client {
namespace {

const QString kDanger = QStringLiteral("#e5484d");
const QString kWarning = QStringLiteral("#e0a526");
const QString kSuccess = QStringLiteral("#2fb36d");

QString colored(const QString& color, const QString& text) {
    return QStringLiteral("<span style=\"color:%1\">%2</span>").arg(color, text.toHtmlEscaped());
}

QLabel* errorLabel(QWidget* parent, const QString& name) {
    auto* label = new QLabel(parent);
    label->setObjectName(name);
    label->setWordWrap(true);
    label->setStyleSheet(QStringLiteral("color: %1;").arg(kDanger));
    label->hide();
    return label;
}

}  // namespace

JoinPage::JoinPage(ClientController& controller, QWidget* parent) : QWidget(parent), controller_(controller) {
    build();
    connect(&controller_, &ClientController::robotChanged, this, &JoinPage::refreshRobot);
    connect(&controller_, &ClientController::presentationChanged, this, &JoinPage::refreshState);
    if (!robot_->text().trimmed().isEmpty() && controller_.robot().path.empty()) controller_.chooseRobot(robot_->text());
    refreshRobot();
    refreshState();
}

QLabel* JoinPage::fieldError(JoinField field) const { return errors_[static_cast<int>(field)]; }

void JoinPage::build() {
    setObjectName(QStringLiteral("client.joinPage"));
    const ClientPrefs& prefs = controller_.prefs();

    auto* title = new QLabel(tr("Join a practice match"), this);
    QFont titleFont = title->font();
    titleFont.setBold(true);
    titleFont.setPointSizeF(titleFont.pointSizeF() * 1.6);
    title->setFont(titleFont);
    auto* subtitle = ui::mutedLabel(
        tr("Paste the invite the host shared with you, choose your robot description and join. Your robot drives on the host's field "
           "and responds to your gamepad or keyboard immediately."),
        this);

    invite_ = new QLineEdit(this);
    invite_->setObjectName(QStringLiteral("client.invite"));
    invite_->setPlaceholderText(QStringLiteral("XXXXX-XXXXX-XXXXX-XXXXX"));
    invite_->setClearButtonEnabled(true);
    invite_->setEchoMode(QLineEdit::Normal);
    invite_->setToolTip(tr("The invite the host shows in View > Practice match. Any case; dashes are optional. It is never saved."));
    errors_[static_cast<int>(JoinField::Invite)] = errorLabel(this, QStringLiteral("client.inviteError"));

    name_ = new QLineEdit(QString::fromStdString(prefs.displayName), this);
    name_->setObjectName(QStringLiteral("client.name"));
    name_->setMaxLength(static_cast<int>(sdk::match::kMaxNameBytes));
    name_->setToolTip(tr("The name other players see next to your robot."));
    errors_[static_cast<int>(JoinField::DisplayName)] = errorLabel(this, QStringLiteral("client.nameError"));

    robot_ = new QLineEdit(QString::fromStdString(prefs.robotPath), this);
    robot_->setObjectName(QStringLiteral("client.robotPath"));
    robot_->setPlaceholderText(tr("The robot.json of your project (.wpiemu/robot.json)"));
    browse_ = new QPushButton(tr("Browse…"), this);
    browse_->setObjectName(QStringLiteral("client.browse"));
    auto* robotRow = new QHBoxLayout;
    robotRow->setContentsMargins(0, 0, 0, 0);
    robotRow->addWidget(robot_, 1);
    robotRow->addWidget(browse_);
    robotReport_ = new QLabel(this);
    robotReport_->setObjectName(QStringLiteral("client.robotReport"));
    robotReport_->setWordWrap(true);
    robotReport_->setTextFormat(Qt::RichText);
    errors_[static_cast<int>(JoinField::Robot)] = errorLabel(this, QStringLiteral("client.robotError"));

    advancedToggle_ = new QToolButton(this);
    advancedToggle_->setObjectName(QStringLiteral("client.advanced"));
    advancedToggle_->setText(tr("Advanced"));
    advancedToggle_->setCheckable(true);
    advancedToggle_->setToolButtonStyle(Qt::ToolButtonTextBesideIcon);
    advancedToggle_->setArrowType(Qt::RightArrow);
    advancedToggle_->setAutoRaise(true);
    advanced_ = new QWidget(this);
    advanced_->setObjectName(QStringLiteral("client.advancedPanel"));
    advanced_->setVisible(false);

    direct_ = new QLineEdit(QString::fromStdString(prefs.directAddress), advanced_);
    direct_->setObjectName(QStringLiteral("client.direct"));
    direct_->setPlaceholderText(tr("Optional: host:port for a direct connection (LAN or forwarded port)"));
    direct_->setClearButtonEnabled(true);
    direct_->setToolTip(tr("Leave empty to connect through the relay. A direct connection needs the host's address and UDP port "
                           "(47300 unless the host changed it) and the same invite."));
    errors_[static_cast<int>(JoinField::DirectAddress)] = errorLabel(advanced_, QStringLiteral("client.directError"));

    relayChoice_ = new QComboBox(advanced_);
    relayChoice_->setObjectName(QStringLiteral("client.relayChoice"));
    relayChoice_->addItem(tr("Production (default)"), static_cast<int>(RelayChoice::Production));
    relayChoice_->addItem(tr("Development"), static_cast<int>(RelayChoice::Development));
    relayChoice_->addItem(tr("Custom…"), static_cast<int>(RelayChoice::Custom));
    relayChoice_->setCurrentIndex(relayChoice_->findData(static_cast<int>(prefs.relay)));
    customRelay_ = new QLineEdit(QString::fromStdString(prefs.customRelay), advanced_);
    customRelay_->setObjectName(QStringLiteral("client.customRelay"));
    customRelay_->setPlaceholderText(tr("host:port of your relay"));
    relayAddress_ = ui::captionLabel(QString(), advanced_);
    relayAddress_->setObjectName(QStringLiteral("client.relayAddress"));
    errors_[static_cast<int>(JoinField::Relay)] = errorLabel(advanced_, QStringLiteral("client.relayError"));

    deadband_ = new QDoubleSpinBox(advanced_);
    deadband_->setObjectName(QStringLiteral("client.deadband"));
    deadband_->setRange(0.0, 90.0);
    deadband_->setDecimals(0);
    deadband_->setSuffix(QStringLiteral(" %"));
    deadband_->setValue(static_cast<double>(prefs.drive.deadband) * 100.0);
    deadband_->setToolTip(tr("Stick travel around the centre that is ignored."));
    linearScale_ = new QDoubleSpinBox(advanced_);
    linearScale_->setObjectName(QStringLiteral("client.speedScale"));
    linearScale_->setRange(5.0, 100.0);
    linearScale_->setDecimals(0);
    linearScale_->setSuffix(QStringLiteral(" %"));
    linearScale_->setValue(static_cast<double>(prefs.drive.linearScale) * 100.0);
    linearScale_->setToolTip(tr("The share of your robot's top speed that a full stick deflection commands."));
    fieldRelative_ = new QCheckBox(tr("Drive relative to the field"), advanced_);
    fieldRelative_->setObjectName(QStringLiteral("client.fieldRelative"));
    fieldRelative_->setChecked(prefs.drive.fieldRelative);

    auto* advancedForm = new QFormLayout(advanced_);
    advancedForm->setContentsMargins(0, 0, 0, 0);
    advancedForm->addRow(tr("Direct address"), direct_);
    advancedForm->addRow(QString(), errors_[static_cast<int>(JoinField::DirectAddress)]);
    advancedForm->addRow(tr("Relay"), relayChoice_);
    advancedForm->addRow(QString(), customRelay_);
    advancedForm->addRow(QString(), relayAddress_);
    advancedForm->addRow(QString(), errors_[static_cast<int>(JoinField::Relay)]);
    advancedForm->addRow(tr("Stick deadband"), deadband_);
    advancedForm->addRow(tr("Top speed"), linearScale_);
    advancedForm->addRow(QString(), fieldRelative_);

    auto* form = new QFormLayout;
    form->setLabelAlignment(Qt::AlignRight);
    form->addRow(tr("Invite"), invite_);
    form->addRow(QString(), errors_[static_cast<int>(JoinField::Invite)]);
    form->addRow(tr("Your name"), name_);
    form->addRow(QString(), errors_[static_cast<int>(JoinField::DisplayName)]);
    form->addRow(tr("Robot"), robotRow);
    form->addRow(QString(), robotReport_);
    form->addRow(QString(), errors_[static_cast<int>(JoinField::Robot)]);

    status_ = new QLabel(this);
    status_->setObjectName(QStringLiteral("client.status"));
    status_->setWordWrap(true);
    status_->setTextFormat(Qt::RichText);
    status_->hide();
    busy_ = new QProgressBar(this);
    busy_->setRange(0, 0);
    busy_->setTextVisible(false);
    busy_->setMaximumHeight(6);
    busy_->hide();

    join_ = new QPushButton(tr("Join match"), this);
    join_->setObjectName(QStringLiteral("client.join"));
    join_->setDefault(true);
    cancel_ = new QPushButton(tr("Cancel"), this);
    cancel_->setObjectName(QStringLiteral("client.cancel"));
    cancel_->hide();
    auto* buttons = new QHBoxLayout;
    buttons->addStretch(1);
    buttons->addWidget(cancel_);
    buttons->addWidget(join_);

    auto* card = new QWidget(this);
    card->setMaximumWidth(680);
    auto* cardLayout = new QVBoxLayout(card);
    cardLayout->setContentsMargins(0, 0, 0, 0);
    cardLayout->setSpacing(10);
    cardLayout->addWidget(title);
    cardLayout->addWidget(subtitle);
    cardLayout->addSpacing(6);
    cardLayout->addLayout(form);
    cardLayout->addWidget(advancedToggle_, 0, Qt::AlignLeft);
    cardLayout->addWidget(advanced_);
    cardLayout->addWidget(status_);
    cardLayout->addWidget(busy_);
    cardLayout->addLayout(buttons);
    cardLayout->addStretch(1);

    auto* outer = new QHBoxLayout(this);
    outer->setContentsMargins(32, 28, 32, 24);
    outer->addStretch(1);
    outer->addWidget(card, 100);
    outer->addStretch(1);

    connect(browse_, &QPushButton::clicked, this, &JoinPage::browse);
    connect(join_, &QPushButton::clicked, this, &JoinPage::submit);
    connect(invite_, &QLineEdit::returnPressed, this, &JoinPage::submit);
    connect(cancel_, &QPushButton::clicked, &controller_, &ClientController::leave);
    connect(robot_, &QLineEdit::editingFinished, this, &JoinPage::onRobotEdited);
    connect(advancedToggle_, &QToolButton::toggled, this, [this](bool open) {
        advanced_->setVisible(open);
        advancedToggle_->setArrowType(open ? Qt::DownArrow : Qt::RightArrow);
    });
    connect(relayChoice_, &QComboBox::currentIndexChanged, this, &JoinPage::refreshRelay);
    connect(customRelay_, &QLineEdit::textChanged, this, &JoinPage::refreshRelay);
    for (QLineEdit* edit : {invite_, name_, direct_, customRelay_, robot_})
        connect(edit, &QLineEdit::textChanged, this, &JoinPage::refreshErrors);
    connect(deadband_, &QDoubleSpinBox::valueChanged, this, &JoinPage::storeControls);
    connect(linearScale_, &QDoubleSpinBox::valueChanged, this, &JoinPage::storeControls);
    connect(fieldRelative_, &QCheckBox::toggled, this, &JoinPage::storeControls);

    // Someone who set a non-default relay or a direct address before finds it open.
    if (!prefs.directAddress.empty() || prefs.relay != RelayChoice::Production) advancedToggle_->setChecked(true);
    refreshRelay();
}

JoinFormInput JoinPage::input() const {
    JoinFormInput in;
    in.invite = invite_->text().toStdString();
    in.displayName = name_->text().toStdString();
    in.robotPath = robot_->text().toStdString();
    in.directAddress = direct_->text().toStdString();
    in.relay = static_cast<RelayChoice>(relayChoice_->currentData().toInt());
    in.customRelay = customRelay_->text().toStdString();
    return in;
}

void JoinPage::setInvite(const QString& text) { invite_->setText(text); }

void JoinPage::browse() {
    const QString current = robot_->text().trimmed();
    const QString start = current.isEmpty() ? QDir::homePath() : QFileInfo(current).absolutePath();
    const QString path = QFileDialog::getOpenFileName(this, tr("Choose your robot.json"), start, tr("Robot description (*.json);;All files (*)"));
    if (path.isEmpty()) return;
    robot_->setText(path);
    controller_.chooseRobot(path);
}

void JoinPage::onRobotEdited() { controller_.chooseRobot(robot_->text()); }

void JoinPage::submit() {
    showErrors_ = true;
    const JoinValidation result = controller_.join(input());
    showErrors(result.errors);
    if (!result.ok()) {
        status_->hide();
        return;
    }
    controller_.tick();   // shows Connecting at once
}

void JoinPage::showErrors(const std::vector<JoinFieldError>& errors) {
    for (QLabel* label : errors_) {
        if (label == nullptr) continue;
        label->hide();
        label->clear();
    }
    bool advancedError = false;
    for (const JoinFieldError& e : errors) {
        QLabel* label = errors_[static_cast<int>(e.field)];
        label->setText(fieldErrorText(e));
        label->show();
        advancedError = advancedError || e.field == JoinField::DirectAddress || e.field == JoinField::Relay;
    }
    if (advancedError) advancedToggle_->setChecked(true);
}

void JoinPage::refreshErrors() {
    if (!showErrors_) return;
    showErrors(controller_.validate(input()).errors);
}

void JoinPage::refreshRelay() {
    const JoinFormInput in = input();
    const bool custom = in.relay == RelayChoice::Custom;
    customRelay_->setVisible(custom);
    const std::string address = relayAddressOf(in, controller_.relayPresets());
    relayAddress_->setText(address.empty() ? (custom ? QString() : tr("This build does not know this relay's address yet; choose Custom…."))
                                           : QString::fromStdString(address));
    controller_.setPrefs([&] {
        ClientPrefs p = controller_.prefs();
        p.relay = in.relay;
        p.customRelay = in.customRelay;
        return p;
    }());
    refreshErrors();
}

void JoinPage::storeControls() {
    ClientPrefs p = controller_.prefs();
    p.drive.deadband = static_cast<float>(deadband_->value() / 100.0);
    p.drive.linearScale = static_cast<float>(linearScale_->value() / 100.0);
    p.drive.fieldRelative = fieldRelative_->isChecked();
    controller_.setPrefs(p);
}

void JoinPage::refreshRobot() {
    const RobotFileReport& r = controller_.robot();
    if (robot_->text().trimmed() != QString::fromStdString(r.path) && !r.path.empty()) {
        const QSignalBlocker block(robot_);
        robot_->setText(QString::fromStdString(r.path));
    }
    QStringList lines;
    if (r.path.empty()) {
        robotReport_->hide();
        return;
    }
    if (r.ok) {
        lines << colored(kSuccess, tr("✔ %1: %2 wheels, %3 zones%4")
                                       .arg(QString::fromStdString(r.robotName))
                                       .arg(r.wheelCount)
                                       .arg(r.zoneCount)
                                       .arg(r.gamePluginId.empty() ? QString() : QStringLiteral(", ") + QString::fromStdString(r.gamePluginId)));
        if (!r.gamePluginId.empty() && !r.gameKnown)
            lines << colored(kWarning, tr("This client has no game plugin “%1”; the host will refuse a robot for a game it does not play.")
                                           .arg(QString::fromStdString(r.gamePluginId)));
        if (r.gamePluginId.empty())
            lines << colored(kWarning, tr("This robot description names no game; the host plays one and will refuse it."));
        if (!r.clamps.empty()) {
            lines << colored(kWarning, tr("The host keeps robots within plausible FRC limits and will adjust:"));
            for (const std::string& c : r.clamps) lines << colored(kWarning, QStringLiteral("• ") + QString::fromStdString(c));
        }
    } else {
        lines << colored(kDanger, tr("This file cannot be used:"));
        for (const std::string& e : r.errors) lines << colored(kDanger, QStringLiteral("• ") + QString::fromStdString(e));
    }
    robotReport_->setText(lines.join(QStringLiteral("<br>")));
    robotReport_->show();
    refreshErrors();
}

void JoinPage::refreshState() {
    const Presentation& p = controller_.presentation();
    const bool joining = p.joining;
    for (QWidget* w : {static_cast<QWidget*>(invite_), static_cast<QWidget*>(name_), static_cast<QWidget*>(robot_),
                       static_cast<QWidget*>(browse_), static_cast<QWidget*>(direct_), static_cast<QWidget*>(relayChoice_),
                       static_cast<QWidget*>(customRelay_), static_cast<QWidget*>(join_)})
        w->setEnabled(!joining);
    cancel_->setVisible(joining);
    busy_->setVisible(joining);

    if (p.banner == Banner::None || p.screen != Screen::Join) {
        status_->hide();
        return;
    }
    const QString color = p.tone == runview::Tone::Error ? kDanger : (p.tone == runview::Tone::Warning ? kWarning : QString());
    QString text = QStringLiteral("<b>%1</b>").arg(bannerTitle(p.banner).toHtmlEscaped());
    if (const QString detail = bannerDetail(p); !detail.isEmpty()) text += QStringLiteral("<br>") + detail.toHtmlEscaped();
    status_->setText(color.isEmpty() ? text : QStringLiteral("<span style=\"color:%1\">%2</span>").arg(color, text));
    status_->show();
}

}  // namespace wpiemu::app::client
