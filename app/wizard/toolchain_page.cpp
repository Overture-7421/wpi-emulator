#include "wizard/toolchain_page.h"

#include <QHBoxLayout>
#include <QMessageBox>

namespace wpiemu::app {

namespace {

struct StatusLook {
    const char* icon;
    IconTone tone;
};

StatusLook lookFor(CheckStatus status) {
    switch (status) {
        case CheckStatus::Ok: return {"check-circle", IconTone::Success};
        case CheckStatus::Warning: return {"alert-triangle", IconTone::Warning};
        case CheckStatus::Missing: return {"x-circle", IconTone::Error};
        case CheckStatus::NotApplicable: break;
    }
    return {"minus-circle", IconTone::Muted};
}

}  // namespace

ToolchainPage::ToolchainPage(WizardViewModel& model, const UiResources& ui, QWidget* parent)
    : WizardPage(parent),
      model_(model),
      ui_(ui),
      recheckButton_(new QPushButton(tr("Re-check"))),
      busy_(new QProgressBar),
      busyLabel_(makeLabel(tr("Checking your machine…"), "muted")),
      summary_(new Banner(ui)),
      coreOnly_(new Banner(ui)),
      rowsLayout_(new QVBoxLayout) {
    auto* root = new QVBoxLayout(this);
    root->setContentsMargins(0, 0, 0, 0);
    root->setSpacing(12);

    auto* top = new QHBoxLayout;
    busy_->setRange(0, 0);
    busy_->setTextVisible(false);
    busy_->setFixedHeight(6);
    auto* busyBox = new QVBoxLayout;
    busyBox->setSpacing(4);
    busyBox->addWidget(busyLabel_);
    busyBox->addWidget(busy_);
    top->addLayout(busyBox, 1);
    top->addWidget(recheckButton_);
    root->addLayout(top);

    root->addWidget(summary_);
    rowsLayout_->setSpacing(8);
    root->addLayout(rowsLayout_);
    root->addWidget(coreOnly_);
    root->addStretch(1);

    connect(recheckButton_, &QPushButton::clicked, &model_, &WizardViewModel::recheckToolchain);
    connect(&model_, &WizardViewModel::toolchainChanged, this, &ToolchainPage::refresh);
    connect(&model_, &WizardViewModel::javaChanged, this, &ToolchainPage::refresh);
    consent_ = [this](const QString& question) {
        return QMessageBox::question(this, tr("Get Java"), question, QMessageBox::Yes | QMessageBox::No, QMessageBox::Yes) == QMessageBox::Yes;
    };
    connect(&ui_, &UiResources::themeChanged, this, [this] {
        for (Row& r : rows_) applyIcon(r);
        recheckButton_->setIcon(ui_.icon(QStringLiteral("refresh")));
    });
    recheckButton_->setIcon(ui_.icon(QStringLiteral("refresh")));
    refresh();
}

void ToolchainPage::onEnter() {
    if (!model_.toolchainChecked() && !model_.toolchainRunning()) model_.recheckToolchain();
    refresh();
}

CheckStatus ToolchainPage::rowStatus(const QString& id) const {
    for (const Row& r : rows_)
        if (r.id == id) return r.status;
    return CheckStatus::Warning;
}

QPushButton* ToolchainPage::rowAction(const QString& id) const {
    for (const Row& r : rows_)
        if (r.id == id) return r.action;
    return nullptr;
}

void ToolchainPage::onGetJava() {
    if (model_.javaBusy()) {
        model_.cancelJava();
        return;
    }
    const QString question =
        tr("WPI Emulator will download Java %1 (about %2 MB) from github.com (Eclipse Temurin) and keep it in\n%3\n\n"
           "The download is checked against a published checksum and needs no administrator rights. It is used only because no other "
           "Java that can run this project was found; your own Java and WPILib are not touched.\n\nDownload it now?")
            .arg(model_.javaMajor())
            .arg((model_.javaDownloadBytes() + 500000) / 1000000)
            .arg(model_.javaFolder());
    if (consent_ && consent_(question)) model_.getJava();
}

void ToolchainPage::clearRows() {
    for (Row& r : rows_) {
        rowsLayout_->removeWidget(r.frame);
        r.frame->deleteLater();
    }
    rows_.clear();
}

void ToolchainPage::applyIcon(Row& row) const {
    const StatusLook look = lookFor(row.status);
    row.icon->setPixmap(ui_.icon(QString::fromLatin1(look.icon), look.tone).pixmap(22, 22));
}

void ToolchainPage::refresh() {
    const bool checking = model_.toolchainRunning();
    const bool downloading = model_.javaBusy();
    const bool running = checking || downloading;
    busy_->setVisible(running);
    busyLabel_->setVisible(running);
    if (downloading && model_.javaPercent() >= 0) {
        busy_->setRange(0, 100);
        busy_->setValue(model_.javaPercent());
        busyLabel_->setText(tr("Downloading Java %1… %2%").arg(model_.javaMajor()).arg(model_.javaPercent()));
    } else {
        busy_->setRange(0, 0);
        busyLabel_->setText(downloading ? tr("Checking and unpacking Java %1…").arg(model_.javaMajor()) : tr("Checking your machine…"));
    }
    recheckButton_->setEnabled(!running && model_.projectValid());

    clearRows();
    const ToolchainReport& report = model_.toolchain();
    if (checking || !model_.toolchainChecked()) {
        summary_->clear();
        coreOnly_->clear();
        return;
    }

    for (const ToolchainCheck& check : report.checks) {
        Row row;
        row.id = check.id;
        row.status = check.status;
        row.frame = makeCard();
        auto* layout = new QHBoxLayout(row.frame);
        layout->setContentsMargins(14, 10, 14, 10);
        layout->setSpacing(12);
        row.icon = new QLabel;
        row.icon->setFixedSize(22, 22);
        auto* text = new QVBoxLayout;
        text->setSpacing(2);
        row.title = makeLabel(check.title, "section");
        row.detail = makeLabel(check.detail, "muted", nullptr, true);
        row.detail->setTextInteractionFlags(Qt::TextSelectableByMouse);
        text->addWidget(row.title);
        text->addWidget(row.detail);
        if (!check.fixHint.isEmpty() && (check.status == CheckStatus::Missing || check.status == CheckStatus::Warning)) {
            row.hint = makeLabel(tr("How to fix: %1").arg(check.fixHint), "muted", nullptr, true);
            text->addWidget(row.hint);
        }
        if (check.fixAction == FixAction::GetJava && check.status == CheckStatus::Missing && model_.javaDownloadAvailable()) {
            row.action = new QPushButton(downloading ? tr("Cancel") : tr("Get Java %1").arg(model_.javaMajor()));
            row.action->setObjectName(QStringLiteral("toolchain.getJava"));
            connect(row.action, &QPushButton::clicked, this, &ToolchainPage::onGetJava);
            text->addWidget(row.action, 0, Qt::AlignLeft);
        }
        layout->addWidget(row.icon, 0, Qt::AlignTop);
        layout->addLayout(text, 1);
        row.frame->setAccessibleName(QStringLiteral("%1: %2").arg(check.title, check.detail));
        applyIcon(row);
        rowsLayout_->addWidget(row.frame);
        rows_.push_back(row);
    }

    if (const QString failure = model_.javaError(); !failure.isEmpty()) summary_->setMessage(BannerKind::Error, tr("Could not get Java: %1").arg(failure));
    else if (report.canRunRobot()) summary_->setMessage(BannerKind::Success, tr("Everything needed to launch your robot was found."));
    else summary_->setMessage(BannerKind::Warning, tr("Something needed to launch the robot is missing. You can finish the setup now and fix it before running."));

    if (report.coreOnly() && model_.projectValid())
        coreOnly_->setMessage(BannerKind::Info, tr("Core-only mode: this season has no game module yet, so the field and game pieces are not simulated. Everything else works."));
    else
        coreOnly_->clear();
}

}  // namespace wpiemu::app
