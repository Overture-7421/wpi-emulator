#include "model/editor_document.h"

#include <QDateTime>

#include "model/config_ops.h"
#include "model/property_groups.h"
#include "model/tr.h"

namespace wpiemu::app::editor {

namespace {
constexpr int kMergeCommandId = 0x5701;
constexpr qint64 kMergeWindowMs = 1500;
}  // namespace

ConfigEditCommand::ConfigEditCommand(EditorDocument& doc, const QString& text, sdk::RobotConfig before, sdk::RobotConfig after,
                                     QString mergeKey)
    : QUndoCommand(text),
      doc_(doc),
      before_(std::move(before)),
      after_(std::move(after)),
      mergeKey_(std::move(mergeKey)),
      stampMs_(QDateTime::currentMSecsSinceEpoch()) {}

void ConfigEditCommand::undo() { doc_.applySnapshot(before_); }
void ConfigEditCommand::redo() {
    const bool authoritative = !firstRedo_;
    firstRedo_ = false;
    doc_.applySnapshot(after_, authoritative);
}
int ConfigEditCommand::id() const { return mergeKey_.isEmpty() ? -1 : kMergeCommandId; }

bool ConfigEditCommand::mergeWith(const QUndoCommand* other) {
    if (other->id() != kMergeCommandId || id() != kMergeCommandId) return false;
    const auto* o = static_cast<const ConfigEditCommand*>(other);
    if (o->mergeKey_ != mergeKey_) return false;
    if (!mergeKey_.startsWith(QStringLiteral("drag:")) && o->stampMs_ - stampMs_ > kMergeWindowMs) return false;
    after_ = o->after_;
    stampMs_ = o->stampMs_;
    return true;
}

EditorDocument::EditorDocument(QObject* parent) : QObject(parent) { undo_.setUndoLimit(300); }

void EditorDocument::load(sdk::RobotConfig config) {
    undo_.blockSignals(true);
    undo_.clear();
    undo_.blockSignals(false);
    config_ = std::move(config);
    ops::normalizeOrder(config_);
    Q_EMIT changed(true);
}

void EditorDocument::setCatalog(EditorCatalog catalog) {
    catalog_ = std::move(catalog);
    Q_EMIT catalogChanged();
    Q_EMIT changed(true);
}

void EditorDocument::applySnapshot(const sdk::RobotConfig& config, bool authoritative) {
    config_ = config;
    Q_EMIT changed(authoritative);
    Q_EMIT modified();
}

bool EditorDocument::edit(const QString& text, const std::function<bool(sdk::RobotConfig&)>& mutator, const QString& mergeKey) {
    sdk::RobotConfig after = config_;
    if (!mutator(after)) return false;
    undo_.push(new ConfigEditCommand(*this, text, config_, std::move(after), mergeKey));
    return true;
}

QString EditorDocument::addMechanism(const QString& type, const sdk::MechanismAttachment& attachment, const ops::MechanismVariant& variant) {
    const sdk::MechanismTemplateInfo* info = catalog_.findTemplate(type.toStdString());
    if (info == nullptr) return {};
    std::string id;
    edit(ETR("Add %1").arg(QString::fromStdString(info->displayName)), [&](sdk::RobotConfig& c) {
        id = ops::addMechanism(c, *info, attachment, variant);
        return true;
    });
    return QString::fromStdString(id);
}

bool EditorDocument::removeMechanism(const QString& id) {
    const std::string sid = id.toStdString();
    const sdk::MechanismConfig* m = config_.findMechanism(sid);
    if (m == nullptr) return false;
    const QString name = QString::fromStdString(ops::displayNameFor(*m, catalog_));
    return edit(ETR("Remove %1").arg(name), [&](sdk::RobotConfig& c) { return ops::removeMechanism(c, catalog_, sid); });
}

QString EditorDocument::duplicateMechanism(const QString& id) {
    const std::string sid = id.toStdString();
    const sdk::MechanismConfig* m = config_.findMechanism(sid);
    if (m == nullptr) return {};
    const QString name = QString::fromStdString(ops::displayNameFor(*m, catalog_));
    std::string created;
    edit(ETR("Duplicate %1").arg(name), [&](sdk::RobotConfig& c) {
        created = ops::duplicateMechanism(c, sid);
        return !created.empty();
    });
    return QString::fromStdString(created);
}

bool EditorDocument::reparentMechanism(const QString& id, const QString& newParent, const QString& link) {
    const std::string sid = id.toStdString();
    const sdk::MechanismConfig* m = config_.findMechanism(sid);
    if (m == nullptr) return false;
    const QString name = QString::fromStdString(ops::displayNameFor(*m, catalog_));
    return edit(ETR("Attach %1").arg(name), [&](sdk::RobotConfig& c) {
        return ops::reparentMechanism(c, catalog_, sid, newParent.toStdString(), link.toStdString());
    });
}

bool EditorDocument::mapSlot(const sdk::SlotAddress& slot, const sdk::DeviceKey& device) {
    return edit(ETR("Map %1").arg(QString::fromStdString(ops::slotDisplayName(config_, catalog_, slot))),
                [&](sdk::RobotConfig& c) { return ops::setMapping(c, slot, device); });
}

bool EditorDocument::clearSlot(const sdk::SlotAddress& slot) {
    return edit(ETR("Unmap %1").arg(QString::fromStdString(ops::slotDisplayName(config_, catalog_, slot))),
                [&](sdk::RobotConfig& c) { return ops::clearMapping(c, slot); });
}

bool EditorDocument::setProperty(const SelectionRef& sel, const std::string& groupId, const std::string& key, const sdk::Value& value) {
    const QString merge = QStringLiteral("prop:%1:%2:%3").arg(QString::fromStdString(sel.key()), QString::fromStdString(groupId), QString::fromStdString(key));
    return edit(ETR("Edit %1").arg(describeSelection(config_, catalog_, sel)),
                [&](sdk::RobotConfig& c) { return applyProperty(c, catalog_, sel, groupId, key, value); }, merge);
}

bool EditorDocument::setTransform(const SelectionRef& sel, const EditableTransform& t, const QString& dragSession) {
    const QString merge = dragSession.isEmpty() ? QStringLiteral("xform:%1").arg(QString::fromStdString(sel.key()))
                                                : QStringLiteral("drag:%1").arg(dragSession);
    return edit(ETR("Transform %1").arg(describeSelection(config_, catalog_, sel)),
                [&](sdk::RobotConfig& c) { return writeTransform(c, catalog_, sel, t); }, merge);
}

QString EditorDocument::addWheel() {
    std::string name;
    edit(ETR("Add wheel"), [&](sdk::RobotConfig& c) {
        name = ops::addWheel(c);
        return true;
    });
    return QString::fromStdString(name);
}

bool EditorDocument::removeWheel(const QString& name) {
    return edit(ETR("Remove wheel %1").arg(name), [&](sdk::RobotConfig& c) { return ops::removeWheel(c, name.toStdString()); });
}

int EditorDocument::addPrimitive(const QString& owner, sdk::PrimitiveKind kind) {
    int index = -1;
    edit(ETR("Add shape"), [&](sdk::RobotConfig& c) {
        index = ops::addPrimitive(c, owner.toStdString(), kind);
        return index >= 0;
    });
    return index;
}

bool EditorDocument::removePrimitive(const QString& owner, int index) {
    return edit(ETR("Remove shape"), [&](sdk::RobotConfig& c) { return ops::removePrimitive(c, owner.toStdString(), index); });
}

int EditorDocument::duplicatePrimitive(const QString& owner, int index) {
    int created = -1;
    edit(ETR("Duplicate shape"), [&](sdk::RobotConfig& c) {
        created = ops::duplicatePrimitive(c, owner.toStdString(), index);
        return created >= 0;
    });
    return created;
}

}  // namespace wpiemu::app::editor
