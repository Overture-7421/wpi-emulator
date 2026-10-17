#include "wizard/robot_page.h"

#include <QHBoxLayout>
#include <QScrollArea>
#include <QSignalBlocker>
#include <QVBoxLayout>

#include <algorithm>
#include <cmath>

namespace wpiemu::app {

namespace {

QToolButton* makeSegment(const QString& text, const char* position) {
    auto* b = new QToolButton;
    b->setText(text);
    b->setCheckable(true);
    b->setProperty("segment", true);
    b->setProperty("segmentPos", QString::fromLatin1(position));
    b->setCursor(Qt::PointingHandCursor);
    b->setToolButtonStyle(Qt::ToolButtonTextOnly);
    return b;
}

QWidget* segmentRow(std::initializer_list<QToolButton*> buttons) {
    auto* w = new QWidget;
    auto* l = new QHBoxLayout(w);
    l->setContentsMargins(0, 0, 0, 0);
    l->setSpacing(0);
    for (QToolButton* b : buttons) l->addWidget(b);
    l->addStretch(1);
    return w;
}

void setIfDifferent(QDoubleSpinBox* spin, double value) {
    if (std::abs(spin->value() - value) > 1e-4) spin->setValue(value);
}

// Spin box limits rounded inwards to the displayed precision, so the extreme values it offers always
// convert back to a value the model accepts.
void setRoundedRange(QDoubleSpinBox* spin, double lo, double hi) {
    const double scale = std::pow(10.0, spin->decimals());
    spin->setRange(std::ceil(lo * scale) / scale, std::floor(hi * scale) / scale);
}

void markInvalid(QWidget* w, bool invalid) {
    if (w->property("invalid").toBool() == invalid) return;
    w->setProperty("invalid", invalid);
    repolish(w);
}

QFrame* sectionCard(const QString& title, QFormLayout*& formOut) {
    QFrame* card = makeCard();
    auto* outer = new QVBoxLayout(card);
    outer->setContentsMargins(16, 14, 16, 14);
    outer->setSpacing(10);
    outer->addWidget(makeLabel(title, "section"));
    formOut = new QFormLayout;
    formOut->setHorizontalSpacing(16);
    formOut->setVerticalSpacing(9);
    formOut->setFieldGrowthPolicy(QFormLayout::AllNonFixedFieldsGrow);
    formOut->setRowWrapPolicy(QFormLayout::WrapLongRows);
    outer->addLayout(formOut);
    return card;
}

}  // namespace

QDoubleSpinBox* RobotPage::makeSpin(double decimals) {
    auto* s = new QDoubleSpinBox;
    s->setDecimals(static_cast<int>(decimals));
    s->setKeyboardTracking(true);
    s->setAlignment(Qt::AlignRight);
    s->setButtonSymbols(QAbstractSpinBox::UpDownArrows);
    return s;
}

RobotPage::RobotPage(WizardViewModel& model, const UiResources& ui, QWidget* parent)
    : WizardPage(parent),
      model_(model),
      ui_(ui),
      nameEdit_(new QLineEdit),
      kindGroup_(new QButtonGroup(this)),
      swerveButton_(makeSegment(tr("Swerve"), "first")),
      tankButton_(makeSegment(tr("Tank"), "middle")),
      mecanumButton_(makeSegment(tr("Mecanum"), "last")),
      length_(makeSpin(2)),
      width_(makeSpin(2)),
      mass_(makeSpin(1)),
      wheelDiameter_(makeSpin(2)),
      wheelsPerSide_(new QSpinBox),
      wheelCountLabel_(makeLabel({}, "muted")),
      driveMotor_(new QComboBox),
      motorsPerWheel_(new QSpinBox),
      driveRatio_(makeSpin(2)),
      steerMotor_(new QComboBox),
      steerRatio_(makeSpin(2)),
      bumper_(makeSpin(2)),
      redButton_(makeSegment(tr("Red"), "first")),
      blueButton_(makeSegment(tr("Blue"), "last")),
      station_(new QComboBox),
      motorForm_(nullptr),
      dimensionForm_(nullptr),
      banner_(new Banner(ui)),
      schematic_(new RobotSchematicView(ui)) {
    kindGroup_->setExclusive(true);
    kindGroup_->addButton(swerveButton_);
    kindGroup_->addButton(tankButton_);
    kindGroup_->addButton(mecanumButton_);
    auto* allianceGroup = new QButtonGroup(this);
    allianceGroup->addButton(redButton_);
    allianceGroup->addButton(blueButton_);

    swerveButton_->setProperty("kind", QString::fromLatin1(sdk::drivetrain_kind::kSwerve));
    tankButton_->setProperty("kind", QString::fromLatin1(sdk::drivetrain_kind::kTank));
    mecanumButton_->setProperty("kind", QString::fromLatin1(sdk::drivetrain_kind::kMecanum));

    nameEdit_->setPlaceholderText(tr("My robot"));
    nameEdit_->setAccessibleName(tr("Robot name"));
    wheelsPerSide_->setRange(2, 4);
    motorsPerWheel_->setRange(1, 4);
    driveRatio_->setRange(0.5, 100.0);
    steerRatio_->setRange(0.5, 200.0);
    for (const MotorChoice& m : motorCatalog()) {
        driveMotor_->addItem(m.label, m.key);
        steerMotor_->addItem(m.label, m.key);
    }
    for (int s = 1; s <= 3; ++s) station_->addItem(tr("Driver station %1").arg(s), s);

    auto* root = new QHBoxLayout(this);
    root->setContentsMargins(0, 0, 0, 0);
    root->setSpacing(18);

    // ---- form column (scrolls when the window is short) ----
    auto* formHost = new QWidget;
    auto* formLayout = new QVBoxLayout(formHost);
    formLayout->setContentsMargins(0, 0, 8, 0);
    formLayout->setSpacing(12);

    QFormLayout* typeForm = nullptr;
    formLayout->addWidget(sectionCard(tr("Drivetrain"), typeForm));
    typeForm->addRow(tr("Robot name"), nameEdit_);
    typeForm->addRow(tr("Type"), segmentRow({swerveButton_, tankButton_, mecanumButton_}));

    formLayout->addWidget(sectionCard(tr("Size and weight"), dimensionForm_));
    dimensionForm_->addRow(tr("Frame length (front to back)"), length_);
    dimensionForm_->addRow(tr("Frame width"), width_);
    dimensionForm_->addRow(tr("Robot mass"), mass_);
    dimensionForm_->addRow(tr("Bumper thickness"), bumper_);
    dimensionForm_->addRow(tr("Wheel diameter"), wheelDiameter_);
    dimensionForm_->addRow(tr("Wheels per side"), wheelsPerSide_);
    dimensionForm_->addRow(tr("Wheels"), wheelCountLabel_);

    formLayout->addWidget(sectionCard(tr("Motors and gearing"), motorForm_));
    motorForm_->addRow(tr("Drive motor"), driveMotor_);
    motorForm_->addRow(tr("Motors per wheel"), motorsPerWheel_);
    motorForm_->addRow(tr("Drive gear ratio"), driveRatio_);
    motorForm_->addRow(tr("Steer motor"), steerMotor_);
    motorForm_->addRow(tr("Steer gear ratio"), steerRatio_);

    QFormLayout* startForm = nullptr;
    formLayout->addWidget(sectionCard(tr("Alliance"), startForm));
    startForm->addRow(tr("Bumper colour"), segmentRow({redButton_, blueButton_}));
    startForm->addRow(tr("Your driver station"), station_);
    formLayout->addStretch(1);

    auto* scroll = new QScrollArea;
    scroll->setWidgetResizable(true);
    scroll->setFrameShape(QFrame::NoFrame);
    scroll->setHorizontalScrollBarPolicy(Qt::ScrollBarAlwaysOff);
    scroll->setWidget(formHost);
    root->addWidget(scroll, 11);

    // ---- preview column ----
    auto* previewColumn = new QVBoxLayout;
    previewColumn->setSpacing(10);
    previewColumn->addWidget(schematic_, 1);
    previewColumn->addWidget(banner_);
    root->addLayout(previewColumn, 9);

    connect(kindGroup_, &QButtonGroup::buttonClicked, this, [this](QAbstractButton* b) {
        if (!loading_) model_.setDrivetrainKind(b->property("kind").toString().toStdString());
    });
    const auto push = [this] { pushToModel(); };
    connect(nameEdit_, &QLineEdit::textEdited, this, push);
    for (QDoubleSpinBox* s : {length_, width_, mass_, wheelDiameter_, driveRatio_, steerRatio_, bumper_})
        connect(s, &QDoubleSpinBox::valueChanged, this, push);
    for (QSpinBox* s : {wheelsPerSide_, motorsPerWheel_}) connect(s, &QSpinBox::valueChanged, this, push);
    for (QComboBox* c : {driveMotor_, steerMotor_, station_}) connect(c, &QComboBox::currentIndexChanged, this, push);
    connect(redButton_, &QToolButton::clicked, this, push);
    connect(blueButton_, &QToolButton::clicked, this, push);
    connect(&model_, &WizardViewModel::basicsChanged, this, &RobotPage::loadFromModel);

    loadFromModel();
}

QToolButton* RobotPage::drivetrainButton(const std::string& kind) const {
    if (kind == sdk::drivetrain_kind::kTank) return tankButton_;
    if (kind == sdk::drivetrain_kind::kMecanum) return mecanumButton_;
    return swerveButton_;
}

void RobotPage::applyUnits() {
    const UnitFormatter u = model_.units();
    const bool imperial = u.system() == UnitSystem::Imperial;
    const int lengthDecimals = imperial ? 2 : 1;
    const double step = imperial ? 0.25 : 0.5;
    for (QDoubleSpinBox* s : {length_, width_, bumper_, wheelDiameter_}) {
        s->setDecimals(lengthDecimals);
        s->setSingleStep(step);
        s->setSuffix(u.lengthSuffix());
    }
    setRoundedRange(length_, u.lengthToDisplay(0.25), u.lengthToDisplay(1.5));
    setRoundedRange(width_, u.lengthToDisplay(0.25), u.lengthToDisplay(1.5));
    setRoundedRange(bumper_, 0.0, u.lengthToDisplay(0.15));
    setRoundedRange(wheelDiameter_, u.lengthToDisplay(0.04), u.lengthToDisplay(0.25));
    mass_->setSuffix(u.massSuffix());
    setRoundedRange(mass_, u.massToDisplay(5.0), u.massToDisplay(120.0));
    mass_->setSingleStep(imperial ? 1.0 : 0.5);
    driveRatio_->setSingleStep(0.05);
    steerRatio_->setSingleStep(0.1);
}

void RobotPage::updateVisibility() {
    const RobotBasics& b = model_.basics();
    if (auto* label = qobject_cast<QLabel*>(motorForm_->labelForField(motorsPerWheel_)))
        label->setText(b.isTank() ? tr("Motors per powered side") : tr("Motors per wheel"));
    dimensionForm_->setRowVisible(wheelsPerSide_, b.isTank());
    motorForm_->setRowVisible(steerMotor_, b.isSwerve());
    motorForm_->setRowVisible(steerRatio_, b.isSwerve());
}

void RobotPage::loadFromModel() {
    loading_ = true;
    const RobotBasics& b = model_.basics();
    const UnitFormatter u = model_.units();
    applyUnits();

    if (!nameEdit_->hasFocus() || nameEdit_->text() != b.name) nameEdit_->setText(b.name);
    drivetrainButton(b.drivetrain)->setChecked(true);
    setIfDifferent(length_, u.lengthToDisplay(b.frameLengthM));
    setIfDifferent(width_, u.lengthToDisplay(b.frameWidthM));
    setIfDifferent(mass_, u.massToDisplay(b.massKg));
    setIfDifferent(bumper_, u.lengthToDisplay(b.bumperThicknessM));
    setIfDifferent(wheelDiameter_, u.lengthToDisplay(b.wheelDiameterM));
    if (wheelsPerSide_->value() != b.wheelsPerSide) wheelsPerSide_->setValue(b.wheelsPerSide);
    if (motorsPerWheel_->value() != b.motorsPerWheel) motorsPerWheel_->setValue(b.motorsPerWheel);
    setIfDifferent(driveRatio_, b.driveGearRatio);
    setIfDifferent(steerRatio_, b.steerGearRatio);
    if (const int i = driveMotor_->findData(b.driveMotor); i >= 0 && driveMotor_->currentIndex() != i) driveMotor_->setCurrentIndex(i);
    if (const int i = steerMotor_->findData(b.steerMotor); i >= 0 && steerMotor_->currentIndex() != i) steerMotor_->setCurrentIndex(i);
    (b.alliance == Alliance::Red ? redButton_ : blueButton_)->setChecked(true);
    if (const int i = station_->findData(b.station); i >= 0 && station_->currentIndex() != i) station_->setCurrentIndex(i);
    wheelCountLabel_->setText(tr("%1 wheels").arg(b.wheelCount()));
    updateVisibility();
    schematic_->setBasics(b, u);

    // Validation feedback: highlight the offending fields and summarise in the banner.
    const auto& issues = model_.basicsIssues();
    const auto has = [&issues](const char* path) {
        return std::any_of(issues.begin(), issues.end(), [path](const sdk::Issue& i) { return i.severity == sdk::Severity::Error && i.path == path; });
    };
    markInvalid(nameEdit_, has("name"));
    markInvalid(length_, has("frame"));
    markInvalid(width_, has("frame"));
    markInvalid(mass_, has("mass"));
    markInvalid(wheelDiameter_, has("wheel"));
    markInvalid(driveRatio_, has("gearing"));
    markInvalid(steerRatio_, has("steerGearing"));
    markInvalid(bumper_, has("bumper"));

    QString errors, warnings;
    for (const sdk::Issue& i : issues) {
        QString& target = i.severity == sdk::Severity::Error ? errors : warnings;
        if (!target.isEmpty()) target += QLatin1Char('\n');
        target += QString::fromStdString(i.message);
    }
    if (!errors.isEmpty()) banner_->setMessage(BannerKind::Error, errors);
    else if (!warnings.isEmpty()) banner_->setMessage(BannerKind::Warning, warnings);
    else banner_->clear();
    loading_ = false;
}

void RobotPage::pushToModel() {
    if (loading_) return;
    const UnitFormatter u = model_.units();
    RobotBasics b = model_.basics();
    b.name = nameEdit_->text();
    b.frameLengthM = u.lengthFromDisplay(length_->value());
    b.frameWidthM = u.lengthFromDisplay(width_->value());
    b.massKg = u.massFromDisplay(mass_->value());
    b.bumperThicknessM = u.lengthFromDisplay(bumper_->value());
    b.wheelDiameterM = u.lengthFromDisplay(wheelDiameter_->value());
    b.wheelsPerSide = wheelsPerSide_->value();
    b.motorsPerWheel = motorsPerWheel_->value();
    b.driveGearRatio = driveRatio_->value();
    b.steerGearRatio = steerRatio_->value();
    b.driveMotor = driveMotor_->currentData().toString();
    b.steerMotor = steerMotor_->currentData().toString();
    b.alliance = blueButton_->isChecked() ? Alliance::Blue : Alliance::Red;
    b.station = station_->currentData().toInt();
    model_.setBasics(b);
}

}  // namespace wpiemu::app
